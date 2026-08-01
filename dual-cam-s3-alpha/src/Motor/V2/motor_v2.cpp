#include "motor_v2.h"

#include "Motor/V2/parameters/motor_v2_parameters.h"
#include "Motor/servos.h"
#include "Build/Log/cmd_registry.h"
#include "Build/Log/logger.h"
#include "Build/config/target_config.h"
#include "Network/mqtt/mqtt_trigger.h"

#include <ctype.h>

#ifndef CFG_MQTT_TOPIC_MOTOR_STATE
#define CFG_MQTT_TOPIC_MOTOR_STATE CFG_MQTT_TOPIC_ROOT "/motor/state"
#endif

#ifndef CFG_MOTOR_V2_STATUS_STREAM_LEASE_MS
#define CFG_MOTOR_V2_STATUS_STREAM_LEASE_MS 60000UL
#endif

using MotorV2Parameters::StepKind;

struct MotorV2Step
{
    StepKind kind;
    uint32_t durationMs;
    uint32_t channelMask;
    int16_t angles[CFG_SERVO_CHANNELS];
};

struct MotorV2Program
{
    bool used;
    char name[CFG_MOTOR_V2_MAX_NAME_LEN + 1];
    uint8_t stepCount;
    uint32_t defaultLoops;
    MotorV2Step steps[CFG_MOTOR_V2_MAX_STEPS];
};

struct MotorV2Runner
{
    bool active;
    bool stepStarted;
    uint8_t programIndex;
    int8_t pendingProgramIndex; // NEW: tracks background swaps (-1 means none)
    uint8_t stepIndex;
    uint32_t loopsTarget; // 0 means infinite.
    uint32_t loopsDone;
    uint32_t stepStartMs;
    uint32_t stepDurationMs;
};

static MotorV2Program s_programs[CFG_MOTOR_V2_MAX_PROGRAMS];
static MotorV2Runner s_runner;

static String s_activeSession;
static uint32_t s_lastMotionSeq = 0;
static uint32_t s_lastHeartbeatSeq = 0;
static uint32_t s_lastHeartbeatMs = 0;
static uint32_t s_lastMotorStateMs = 0;
static uint32_t s_statusStreamUntilMs = 0;
static bool s_watchdogArmed = false;
static bool s_watchdogExpired = false;
static bool s_motorStateDirty = false;

static void markMotorStateDirty()
{
    s_motorStateDirty = true;
}

static bool statusStreamActive(uint32_t now = millis())
{
    return s_statusStreamUntilMs != 0 && (int32_t)(s_statusStreamUntilMs - now) > 0;
}

static void resetStep(MotorV2Step &step)
{
    step.kind = StepKind::Empty;
    step.durationMs = 0;
    step.channelMask = 0;
    for (uint8_t i = 0; i < CFG_SERVO_CHANNELS; i++)
        step.angles[i] = -1;
}

static void resetProgram(MotorV2Program &program)
{
    program.used = false;
    program.name[0] = '\0';
    program.stepCount = 0;
    program.defaultLoops = 1;
    for (uint8_t i = 0; i < CFG_MOTOR_V2_MAX_STEPS; i++)
        resetStep(program.steps[i]);
}

static String lowerCopy(String value)
{
    value.trim();
    value.toLowerCase();
    return value;
}

static bool startsWithIgnoreCase(const String &value, const char *prefix)
{
    const size_t len = strlen(prefix);
    if (value.length() < len)
        return false;

    for (size_t i = 0; i < len; i++)
    {
        if (tolower((unsigned char)value[i]) != tolower((unsigned char)prefix[i]))
            return false;
    }
    return true;
}

static bool parseLongStrict(const String &raw, long &out)
{
    String text = raw;
    text.trim();
    if (text.length() == 0)
        return false;

    int start = 0;
    if (text[0] == '-')
        start = 1;
    if (start >= (int)text.length())
        return false;

    for (int i = start; i < (int)text.length(); i++)
    {
        if (!isdigit((unsigned char)text[i]))
            return false;
    }

    out = text.toInt();
    return true;
}

static bool parseKeyValueLong(const String &token, const char *key, long &out)
{
    const int eq = token.indexOf('=');
    if (eq <= 0)
        return false;

    String lhs = token.substring(0, eq);
    lhs.trim();
    lhs.toLowerCase();
    if (lhs != key)
        return false;

    return parseLongStrict(token.substring(eq + 1), out);
}

static int splitTokens(const String &line, String *tokens, int maxTokens)
{
    int count = 0;
    int index = 0;

    while (index < (int)line.length())
    {
        while (index < (int)line.length() && isspace((unsigned char)line[index]))
            index++;
        if (index >= (int)line.length())
            break;

        const int start = index;
        while (index < (int)line.length() && !isspace((unsigned char)line[index]))
            index++;

        if (count >= maxTokens)
            return -1;
        tokens[count++] = line.substring(start, index);
    }

    return count;
}

static String firstToken(const String &line)
{
    int index = 0;
    while (index < (int)line.length() && isspace((unsigned char)line[index]))
        index++;

    const int start = index;
    while (index < (int)line.length() && !isspace((unsigned char)line[index]))
        index++;

    return line.substring(start, index);
}

static String stripMotorPrefix(String raw)
{
    raw.trim();
    if (startsWithIgnoreCase(raw, "motor2:"))
        return raw.substring(7);
    if (startsWithIgnoreCase(raw, "motor:"))
        return raw.substring(6);
    return raw;
}

static void jsonEscape(String &out, const String &value)
{
    for (size_t i = 0; i < value.length(); i++)
    {
        const char ch = value[i];
        switch (ch)
        {
        case '\\':
        case '"':
            out += '\\';
            out += ch;
            break;
        case '\n':
            out += "\\n";
            break;
        case '\r':
            out += "\\r";
            break;
        case '\t':
            out += "\\t";
            break;
        default:
            out += ch;
            break;
        }
    }
}

static void publishMotorEvent(const String &event, const String &detail)
{
    String payload;
    payload.reserve(180);
    payload += "{\"type\":\"motor_v2\",\"event\":\"";
    jsonEscape(payload, event);
    payload += "\",\"detail\":\"";
    jsonEscape(payload, detail);
    payload += "\"}";
    mqtt_publish(CFG_MQTT_TOPIC_EVENT, payload);
}

static const char *activeProgramName()
{
    if (!s_runner.active || s_runner.programIndex >= CFG_MOTOR_V2_MAX_PROGRAMS)
        return "";
    return s_programs[s_runner.programIndex].name;
}

static void publishMotorState()
{
    String payload;
    payload.reserve(340);
    payload += "{\"type\":\"motor_v2_state\",\"pca\":";
    payload += String(servos_found() ? 1 : 0);
    payload += ",\"running\":";
    payload += String(s_runner.active ? 1 : 0);
    payload += ",\"program\":\"";
    jsonEscape(payload, activeProgramName());
    payload += "\",\"step\":";
    payload += String(s_runner.active ? s_runner.stepIndex : 0);
    payload += ",\"loops_done\":";
    payload += String(s_runner.loopsDone);
    payload += ",\"loops_target\":";
    payload += String(s_runner.loopsTarget);
    payload += ",\"watchdog\":";
    payload += String(s_watchdogExpired ? 1 : 0);
    payload += ",\"angles\":[";
    for (uint8_t i = 0; i < CFG_SERVO_CHANNELS; i++)
    {
        payload += String(servos_get_angle(i));
        if (i < CFG_SERVO_CHANNELS - 1)
            payload += ',';
    }
    payload += "]}";

    mqtt_publish(CFG_MQTT_TOPIC_MOTOR_STATE, payload);
    s_lastMotorStateMs = millis();
    s_motorStateDirty = false;
}

static void logError(const String &detail)
{
    LOG("[MOTOR2] ERR " + detail);
    publishMotorEvent("error", detail);
}

static int findProgramIndex(const String &name)
{
    for (uint8_t i = 0; i < CFG_MOTOR_V2_MAX_PROGRAMS; i++)
    {
        if (s_programs[i].used && name == s_programs[i].name)
            return i;
    }
    return -1;
}

static int ensureProgram(const String &name, bool clearExisting, String &err)
{
    if (!MotorV2Parameters::validateProgramName(name, err))
        return -1;

    int existing = findProgramIndex(name);
    if (existing >= 0)
    {
        if (clearExisting)
        {
            resetProgram(s_programs[existing]);
            s_programs[existing].used = true;
            name.toCharArray(s_programs[existing].name, sizeof(s_programs[existing].name));
            s_programs[existing].defaultLoops = 1;
        }
        return existing;
    }

    for (uint8_t i = 0; i < CFG_MOTOR_V2_MAX_PROGRAMS; i++)
    {
        if (!s_programs[i].used)
        {
            resetProgram(s_programs[i]);
            s_programs[i].used = true;
            name.toCharArray(s_programs[i].name, sizeof(s_programs[i].name));
            return i;
        }
    }

    err = "program store full";
    return -1;
}

static bool parseLoopsOption(String *tokens,
                             int count,
                             int startIndex,
                             uint32_t defaultValue,
                             uint32_t &loops,
                             String &err)
{
    loops = defaultValue;
    for (int i = startIndex; i < count; i++)
    {
        long raw = 0;
        if (parseKeyValueLong(tokens[i], "loops", raw) ||
            parseKeyValueLong(tokens[i], "loop", raw))
        {
            return MotorV2Parameters::validateLoops(raw, loops, err);
        }
    }
    return true;
}

static bool parseAssignment(const String &token,
                            uint8_t &channel,
                            int16_t &angle,
                            String &err)
{
    const int eq = token.indexOf('=');
    if (eq <= 0 || eq >= (int)token.length() - 1)
    {
        err = "expected channel=angle";
        return false;
    }

    long rawChannel = 0;
    long rawAngle = 0;
    if (!parseLongStrict(token.substring(0, eq), rawChannel))
    {
        err = "invalid channel";
        return false;
    }
    if (!parseLongStrict(token.substring(eq + 1), rawAngle))
    {
        err = "invalid angle";
        return false;
    }

    return MotorV2Parameters::validateChannel(rawChannel, channel, err) &&
           MotorV2Parameters::validateAngle(rawAngle, angle, err);
}

static bool parseStepStatement(const String &statement, MotorV2Step &step, String &err)
{
    resetStep(step);

    String clean = statement;
    clean.trim();
    if (clean.length() == 0)
    {
        err = "empty step";
        return false;
    }

    String tokens[CFG_MOTOR_V2_MAX_TOKENS];
    const int count = splitTokens(clean, tokens, CFG_MOTOR_V2_MAX_TOKENS);
    if (count < 0)
    {
        err = "too many tokens";
        return false;
    }
    if (count == 0)
    {
        err = "empty step";
        return false;
    }

    const String verb = lowerCopy(tokens[0]);

    if (verb == "move")
    {
        if (count < 3)
        {
            err = "move syntax: move <ch> <angle> [ms]";
            return false;
        }

        long rawChannel = 0;
        long rawAngle = 0;
        if (!parseLongStrict(tokens[1], rawChannel) ||
            !parseLongStrict(tokens[2], rawAngle))
        {
            err = "move needs numeric channel and angle";
            return false;
        }

        uint8_t channel = 0;
        int16_t angle = 0;
        if (!MotorV2Parameters::validateChannel(rawChannel, channel, err) ||
            !MotorV2Parameters::validateAngle(rawAngle, angle, err))
            return false;

        uint32_t durationMs = CFG_MOTOR_V2_DEFAULT_MOVE_MS;
        for (int i = 3; i < count; i++)
        {
            long rawMs = 0;
            if (parseKeyValueLong(tokens[i], "ms", rawMs) ||
                parseKeyValueLong(tokens[i], "duration", rawMs) ||
                parseLongStrict(tokens[i], rawMs))
            {
                if (!MotorV2Parameters::validateMoveDuration(rawMs, durationMs, err))
                    return false;
            }
            else
            {
                err = "invalid move duration";
                return false;
            }
        }

        step.kind = StepKind::Move;
        step.durationMs = durationMs;
        step.channelMask = (uint32_t)(1UL << channel);
        step.angles[channel] = angle;
        return true;
    }

    if (verb == "pose")
    {
        if (count < 3)
        {
            err = "pose syntax: pose <ms> <ch=angle>...";
            return false;
        }

        long rawMs = 0;
        int firstAssignment = 2;
        if (parseKeyValueLong(tokens[1], "ms", rawMs) ||
            parseKeyValueLong(tokens[1], "duration", rawMs))
        {
            firstAssignment = 2;
        }
        else if (parseLongStrict(tokens[1], rawMs))
        {
            firstAssignment = 2;
        }
        else
        {
            err = "pose needs duration as the second token";
            return false;
        }

        uint32_t durationMs = 0;
        if (!MotorV2Parameters::validateMoveDuration(rawMs, durationMs, err))
            return false;

        step.kind = StepKind::Pose;
        step.durationMs = durationMs;

        for (int i = firstAssignment; i < count; i++)
        {
            uint8_t channel = 0;
            int16_t angle = 0;
            if (!parseAssignment(tokens[i], channel, angle, err))
                return false;
            step.channelMask |= (uint32_t)(1UL << channel);
            step.angles[channel] = angle;
        }

        if (step.channelMask == 0)
        {
            err = "pose needs at least one channel";
            return false;
        }
        return true;
    }

    if (verb == "wait")
    {
        if (count != 2)
        {
            err = "wait syntax: wait <ms>";
            return false;
        }
        long rawMs = 0;
        uint32_t waitMs = 0;
        if (!parseLongStrict(tokens[1], rawMs) ||
            !MotorV2Parameters::validateWaitDuration(rawMs, waitMs, err))
            return false;

        step.kind = StepKind::Wait;
        step.durationMs = waitMs;
        return true;
    }

    if (verb == "free")
    {
        step.kind = StepKind::Free;
        step.durationMs = 0;

        if (count == 1 || lowerCopy(tokens[1]) == "all")
        {
            step.channelMask = 0;
            return true;
        }
        if (count != 2)
        {
            err = "free syntax: free [all|ch]";
            return false;
        }

        long rawChannel = 0;
        uint8_t channel = 0;
        if (!parseLongStrict(tokens[1], rawChannel) ||
            !MotorV2Parameters::validateChannel(rawChannel, channel, err))
            return false;
        step.channelMask = (uint32_t)(1UL << channel);
        return true;
    }

    err = "unknown step '" + tokens[0] + "'";
    return false;
}

static bool appendStep(MotorV2Program &program, const MotorV2Step &step, String &err)
{
    if (program.stepCount >= CFG_MOTOR_V2_MAX_STEPS)
    {
        err = "program step store full";
        return false;
    }
    program.steps[program.stepCount++] = step;
    markMotorStateDirty();
    return true;
}

static bool validateStepForExecution(const MotorV2Step &step, String &err)
{
    if (step.kind == StepKind::Empty)
    {
        err = "empty step cannot run";
        return false;
    }

    if (step.kind == StepKind::Move || step.kind == StepKind::Pose)
    {
        if (step.channelMask == 0)
        {
            err = "movement step has no channels";
            return false;
        }

        for (uint8_t ch = 0; ch < CFG_SERVO_CHANNELS; ch++)
        {
            if (!(step.channelMask & (1UL << ch)))
                continue;

            uint8_t checkedChannel = 0;
            int16_t checkedAngle = 0;
            if (!MotorV2Parameters::validateChannel(ch, checkedChannel, err) ||
                !MotorV2Parameters::validateAngle(step.angles[ch], checkedAngle, err))
                return false;
        }
    }

    return true;
}

static void holdAllChannels()
{
    for (uint8_t ch = 0; ch < CFG_SERVO_CHANNELS; ch++)
    {
        const int angle = servos_get_angle(ch);
        if (angle >= 0)
            servos_set_timed(ch, angle, 0);
    }
}

static void stopProgram(bool freeOutputs, const String &reason)
{
    const bool wasActive = s_runner.active;
    s_runner.active = false;
    s_runner.stepStarted = false;
    s_watchdogArmed = false;

    if (freeOutputs)
        servos_free_all();
    else
        holdAllChannels();

    if (wasActive)
    {
        LOG("[MOTOR2] Program stopped: " + reason);
        publishMotorEvent("stopped", reason);
    }
    markMotorStateDirty();
}

static bool applyStep(const MotorV2Step &step, uint32_t &durationMsOut)
{
    durationMsOut = 0;

    String err;
    if (!validateStepForExecution(step, err))
    {
        logError(err);
        return false;
    }

    if (!servos_found())
    {
        logError("PCA9685 not found");
        return false;
    }

    if (step.kind == StepKind::Wait)
    {
        durationMsOut = step.durationMs;
        return true;
    }

    if (step.kind == StepKind::Free)
    {
        if (step.channelMask == 0)
            servos_free_all();
        else
        {
            for (uint8_t ch = 0; ch < CFG_SERVO_CHANNELS; ch++)
            {
                if (step.channelMask & (1UL << ch))
                    servos_free(ch);
            }
        }
        return true;
    }

    if (step.kind != StepKind::Move && step.kind != StepKind::Pose)
        return false;

    uint32_t resolvedMs = step.durationMs;
    bool speedLimited = false;

    for (uint8_t ch = 0; ch < CFG_SERVO_CHANNELS; ch++)
    {
        if (!(step.channelMask & (1UL << ch)))
            continue;

        const int target = step.angles[ch];
        const int current = servos_get_angle(ch);
        if (current >= 0)
        {
            bool channelLimited = false;
            const uint32_t channelMs = MotorV2Parameters::resolveMoveDurationMs(
                current, target, step.durationMs, &channelLimited);
            if (channelMs > resolvedMs)
                resolvedMs = channelMs;
            speedLimited = speedLimited || channelLimited;
        }
    }

    uint8_t poseChannels = 0;
    if (step.kind == StepKind::Pose && CFG_MOTOR_V2_POSE_CHANNEL_STAGGER_MS > 0)
    {
        for (uint8_t ch = 0; ch < CFG_SERVO_CHANNELS; ch++)
        {
            if (step.channelMask & (1UL << ch))
                poseChannels++;
        }
    }

    uint8_t poseApplied = 0;
    for (uint8_t ch = 0; ch < CFG_SERVO_CHANNELS; ch++)
    {
        if (!(step.channelMask & (1UL << ch)))
            continue;

        servos_set_timed(ch, step.angles[ch], resolvedMs);

        if (poseChannels > 1 && ++poseApplied < poseChannels)
            delay(CFG_MOTOR_V2_POSE_CHANNEL_STAGGER_MS);
    }

    if (step.kind == StepKind::Pose && CFG_MOTOR_V2_POSE_CHANNEL_STAGGER_MS > 0)
    {
        if (poseChannels > 1)
            durationMsOut = resolvedMs +
                            ((uint32_t)(poseChannels - 1) *
                             CFG_MOTOR_V2_POSE_CHANNEL_STAGGER_MS);
        else
            durationMsOut = resolvedMs;
    }
    else
    {
        durationMsOut = resolvedMs;
    }

    if (speedLimited)
        LOG("[MOTOR2] Move duration expanded by CFG_SERVO_MAX_SPEED_DPS");

    return true;
}

static bool executeDirectStep(const MotorV2Step &step, const String &label)
{
    if (s_runner.active)
        stopProgram(false, "direct command");

    uint32_t duration = 0;
    if (!applyStep(step, duration))
        return false;

    LOG("[MOTOR2] " + label + " accepted" +
        (duration > 0 ? String(" over ") + String(duration) + " ms" : String("")));
    publishMotorEvent("direct", label);
    markMotorStateDirty();
    return true;
}

static bool startProgram(uint8_t programIndex, uint32_t loops)
{
    if (programIndex >= CFG_MOTOR_V2_MAX_PROGRAMS || !s_programs[programIndex].used)
    {
        logError("program not found");
        return false;
    }
    if (s_programs[programIndex].stepCount == 0)
    {
        logError("program has no steps");
        return false;
    }

    if (s_runner.active)
        stopProgram(false, "program replaced");

    s_runner.active = true;
    s_runner.stepStarted = false;
    s_runner.programIndex = programIndex;
    s_runner.pendingProgramIndex = -1; // NEW: clear any pending swaps
    s_runner.stepIndex = 0;
    s_runner.loopsTarget = loops;
    s_runner.loopsDone = 0;
    s_runner.stepStartMs = 0;
    s_runner.stepDurationMs = 0;
    s_watchdogArmed = false;
    s_watchdogExpired = false;

    LOG("[MOTOR2] Program '" + String(s_programs[programIndex].name) +
        "' running loops=" + String(loops));
    publishMotorEvent("program_start", s_programs[programIndex].name);
    markMotorStateDirty();
    return true;
}

static void finishProgram()
{
    LOG("[MOTOR2] Program complete: " + String(activeProgramName()));
    publishMotorEvent("program_complete", activeProgramName());
    s_runner.active = false;
    s_runner.stepStarted = false;
    markMotorStateDirty();
}

static void advanceRunner(uint32_t now)
{
    uint8_t guard = 0;
    while (s_runner.active && guard++ < 6)
    {
        MotorV2Program &program = s_programs[s_runner.programIndex];
        if (s_runner.stepIndex >= program.stepCount)
        {
            s_runner.loopsDone++;
            if (s_runner.loopsTarget != 0 && s_runner.loopsDone >= s_runner.loopsTarget)
            {
                finishProgram();
                return;
            }

            // --- [motor_v2.cpp] Update advanceRunner natural loop block:
            // --- NEW SWAP LOGIC START ---
            if (s_runner.pendingProgramIndex >= 0)
            {
                s_runner.programIndex = s_runner.pendingProgramIndex;
                s_runner.pendingProgramIndex = -1;
                LOG("[MOTOR2] Seamless boundary swap applied -> " + String(s_programs[s_runner.programIndex].name));
                publishMotorEvent("program_swapped", s_programs[s_runner.programIndex].name);
            }
            else
            {
                // ADDED: Emit program_loop to sync the browser's visualization 
                publishMotorEvent("program_loop", program.name);
            }
            // --- NEW SWAP LOGIC END ---

            s_runner.stepIndex = 0;
            s_runner.stepStarted = false;
            markMotorStateDirty();
            continue;
        }

        MotorV2Step &step = program.steps[s_runner.stepIndex];
        if (!s_runner.stepStarted)
        {
            if (!applyStep(step, s_runner.stepDurationMs))
            {
                stopProgram(true, "step rejected");
                return;
            }
            s_runner.stepStartMs = now;
            s_runner.stepStarted = true;
            LOG("[MOTOR2] Step " + String(s_runner.stepIndex + 1) + "/" +
                String(program.stepCount) + " " +
                MotorV2Parameters::stepKindName(step.kind));
            markMotorStateDirty();
        }

        if ((now - s_runner.stepStartMs) < s_runner.stepDurationMs)
            return;

        s_runner.stepIndex++;
        s_runner.stepStarted = false;
        markMotorStateDirty();
    }
}

static bool parseAndRunDirectStep(const String &body)
{
    String err;
    MotorV2Step step;
    if (!parseStepStatement(body, step, err))
    {
        logError(err);
        return false;
    }
    return executeDirectStep(step, MotorV2Parameters::stepKindName(step.kind));
}


static bool handleSwap(const String &body)
{
    String tokens[CFG_MOTOR_V2_MAX_TOKENS];
    const int count = splitTokens(body, tokens, CFG_MOTOR_V2_MAX_TOKENS);
    if (count < 2)
    {
        logError("swap syntax: swap <name> [now]");
        return false;
    }

    if (!s_runner.active)
    {
        logError("cannot swap: no program is currently running");
        return false;
    }

    const int targetIndex = findProgramIndex(tokens[1]);
    if (targetIndex < 0)
    {
        logError("swap target program not found");
        return false;
    }

    bool immediate = (count >= 3 && lowerCopy(tokens[2]) == "now");

    if (immediate)
    {
        // Safe check: if new sequence is shorter than current step index, reset to 0
        if (s_runner.stepIndex >= s_programs[targetIndex].stepCount)
            s_runner.stepIndex = 0;

        s_runner.programIndex = targetIndex;
        s_runner.pendingProgramIndex = -1;
        LOG("[MOTOR2] Immediate swap applied -> " + tokens[1]);
        publishMotorEvent("program_swapped_immediately", tokens[1]);
        markMotorStateDirty();
    }
    else
    {
        s_runner.pendingProgramIndex = targetIndex;
        LOG("[MOTOR2] Swap queued for next cycle boundary -> " + tokens[1]);
        publishMotorEvent("program_swap_queued", tokens[1]);
    }
    
    return true;
}

static bool handleLoad(const String &body)
{
    const int firstSemi = body.indexOf(';');
    String header = (firstSemi == -1) ? body : body.substring(0, firstSemi);
    String tokens[CFG_MOTOR_V2_MAX_TOKENS];
    const int count = splitTokens(header, tokens, CFG_MOTOR_V2_MAX_TOKENS);
    if (count < 2)
    {
        logError("load syntax: load <name> [loops=N]; <steps>; [run]");
        return false;
    }

    String err;
    if (!MotorV2Parameters::validateProgramName(tokens[1], err))
    {
        logError(err);
        return false;
    }

    MotorV2Program parsedProgram;
    resetProgram(parsedProgram);
    parsedProgram.used = true;
    tokens[1].toCharArray(parsedProgram.name, sizeof(parsedProgram.name));

    uint32_t loops = 1;
    if (!parseLoopsOption(tokens, count, 2, 1, loops, err))
    {
        logError(err);
        return false;
    }
    parsedProgram.defaultLoops = loops;

    bool runAfterLoad = false;
    uint32_t runLoops = loops;
    int cursor = (firstSemi == -1) ? -1 : firstSemi + 1;
    while (cursor >= 0 && cursor <= (int)body.length())
    {
        int next = body.indexOf(';', cursor);
        String statement = (next == -1) ? body.substring(cursor) : body.substring(cursor, next);
        statement.trim();
        cursor = (next == -1) ? -1 : next + 1;
        if (statement.length() == 0)
            continue;

        String stepTokens[CFG_MOTOR_V2_MAX_TOKENS];
        const int stepCount = splitTokens(statement, stepTokens, CFG_MOTOR_V2_MAX_TOKENS);
        if (stepCount > 0 && lowerCopy(stepTokens[0]) == "run")
        {
            if (!parseLoopsOption(stepTokens, stepCount, 1, loops, runLoops, err))
            {
                logError(err);
                return false;
            }
            runAfterLoad = true;
            continue;
        }

        MotorV2Step step;
        if (!parseStepStatement(statement, step, err) ||
            !appendStep(parsedProgram, step, err))
        {
            logError(err);
            return false;
        }
    }

    int programIndex = ensureProgram(tokens[1], true, err);
    if (programIndex < 0)
    {
        logError(err);
        return false;
    }

    s_programs[programIndex] = parsedProgram;
    LOG("[MOTOR2] Loaded '" + String(parsedProgram.name) +
        "' steps=" + String(parsedProgram.stepCount) +
        " loops=" + String(parsedProgram.defaultLoops));
    publishMotorEvent("program_loaded", parsedProgram.name);
    markMotorStateDirty();

    if (runAfterLoad)
        return startProgram((uint8_t)programIndex, runLoops);
    return true;
}

static bool handleAdd(const String &body)
{
    String tokens[CFG_MOTOR_V2_MAX_TOKENS];
    const int count = splitTokens(body, tokens, CFG_MOTOR_V2_MAX_TOKENS);
    if (count < 3)
    {
        logError("add syntax: add <name> <step>");
        return false;
    }

    String err;
    int programIndex = ensureProgram(tokens[1], false, err);
    if (programIndex < 0)
    {
        logError(err);
        return false;
    }

    const int stepStart = body.indexOf(tokens[2]);
    if (stepStart < 0)
    {
        logError("missing step");
        return false;
    }

    MotorV2Step step;
    if (!parseStepStatement(body.substring(stepStart), step, err) ||
        !appendStep(s_programs[programIndex], step, err))
    {
        logError(err);
        return false;
    }

    LOG("[MOTOR2] Added step to '" + String(s_programs[programIndex].name) +
        "' count=" + String(s_programs[programIndex].stepCount));
    publishMotorEvent("program_step_added", s_programs[programIndex].name);
    return true;
}

static bool handleSet(const String &body)
{
    String tokens[CFG_MOTOR_V2_MAX_TOKENS];
    const int count = splitTokens(body, tokens, CFG_MOTOR_V2_MAX_TOKENS);
    if (count < 4)
    {
        logError("set syntax: set <name> <index> <step>");
        return false;
    }

    const int programIndex = findProgramIndex(tokens[1]);
    if (programIndex < 0)
    {
        logError("program not found");
        return false;
    }

    long rawIndex = 0;
    if (!parseLongStrict(tokens[2], rawIndex) ||
        rawIndex < 0 ||
        rawIndex >= CFG_MOTOR_V2_MAX_STEPS)
    {
        logError("set index out of range");
        return false;
    }

    MotorV2Program &program = s_programs[programIndex];
    if ((uint8_t)rawIndex > program.stepCount)
    {
        logError("set index cannot skip steps");
        return false;
    }

    const int stepStart = body.indexOf(tokens[3]);
    if (stepStart < 0)
    {
        logError("missing replacement step");
        return false;
    }

    String err;
    MotorV2Step step;
    if (!parseStepStatement(body.substring(stepStart), step, err))
    {
        logError(err);
        return false;
    }

    program.steps[rawIndex] = step;
    if ((uint8_t)rawIndex == program.stepCount)
        program.stepCount++;

    LOG("[MOTOR2] Set step " + String(rawIndex) + " on '" +
        String(program.name) + "'");
    publishMotorEvent("program_step_set", program.name);
    markMotorStateDirty();
    return true;
}

static bool handleOverride(const String &body)
{
    String tokens[CFG_MOTOR_V2_MAX_TOKENS];
    const int count = splitTokens(body, tokens, CFG_MOTOR_V2_MAX_TOKENS);
    if (count < 2)
    {
        logError("override syntax: override <name> [key=value...]");
        return false;
    }

    if (!s_runner.active || String(activeProgramName()) != tokens[1])
    {
        publishMotorEvent("gait_override_ignored", "no matching running gait");
        LOG("[MOTOR2] Gait override ignored for '" + tokens[1] + "'");
        return true;
    }

    String detail = tokens[1];
    for (int i = 2; i < count; i++)
    {
        detail += " ";
        detail += tokens[i];
    }

    LOG("[MOTOR2] Live gait override: " + detail);
    publishMotorEvent("gait_override", detail);
    markMotorStateDirty();
    return true;
}

static bool handleRun(const String &body)
{
    String tokens[CFG_MOTOR_V2_MAX_TOKENS];
    const int count = splitTokens(body, tokens, CFG_MOTOR_V2_MAX_TOKENS);
    if (count < 2)
    {
        logError("run syntax: run <name> [loops=N]");
        return false;
    }

    const int programIndex = findProgramIndex(tokens[1]);
    if (programIndex < 0)
    {
        logError("program not found");
        return false;
    }

    String err;
    uint32_t loops = s_programs[programIndex].defaultLoops;
    if (count >= 3)
    {
        long rawLoops = 0;
        if (parseLongStrict(tokens[2], rawLoops))
        {
            if (!MotorV2Parameters::validateLoops(rawLoops, loops, err))
            {
                logError(err);
                return false;
            }
        }
        else if (!parseLoopsOption(tokens, count, 2, loops, loops, err))
        {
            logError(err);
            return false;
        }
    }
    else if (!parseLoopsOption(tokens, count, 2, loops, loops, err))
    {
        logError(err);
        return false;
    }

    return startProgram((uint8_t)programIndex, loops);
}

static bool handleClear(const String &body)
{
    String tokens[CFG_MOTOR_V2_MAX_TOKENS];
    const int count = splitTokens(body, tokens, CFG_MOTOR_V2_MAX_TOKENS);
    if (count < 2)
    {
        logError("clear syntax: clear <name>");
        return false;
    }

    const int programIndex = findProgramIndex(tokens[1]);
    if (programIndex < 0)
    {
        logError("program not found");
        return false;
    }

    if (s_runner.active && s_runner.programIndex == programIndex)
        stopProgram(false, "program cleared");

    resetProgram(s_programs[programIndex]);
    LOG("[MOTOR2] Program cleared: " + tokens[1]);
    publishMotorEvent("program_cleared", tokens[1]);
    markMotorStateDirty();
    return true;
}

static void appendStepText(String &out, const MotorV2Step &step)
{
    out += MotorV2Parameters::stepKindName(step.kind);
    if (step.kind == StepKind::Wait)
    {
        out += " ";
        out += String(step.durationMs);
        return;
    }
    if (step.kind == StepKind::Free)
    {
        if (step.channelMask == 0)
            out += " all";
        else
        {
            for (uint8_t ch = 0; ch < CFG_SERVO_CHANNELS; ch++)
            {
                if (step.channelMask & (1UL << ch))
                {
                    out += " ";
                    out += String(ch);
                }
            }
        }
        return;
    }

    out += " ";
    out += String(step.durationMs);
    for (uint8_t ch = 0; ch < CFG_SERVO_CHANNELS; ch++)
    {
        if (step.channelMask & (1UL << ch))
        {
            out += " ";
            out += String(ch);
            out += "=";
            out += String(step.angles[ch]);
        }
    }
}

static bool handleShow(const String &body)
{
    String tokens[CFG_MOTOR_V2_MAX_TOKENS];
    const int count = splitTokens(body, tokens, CFG_MOTOR_V2_MAX_TOKENS);
    if (count < 2)
    {
        logError("show syntax: show <name>");
        return false;
    }

    const int programIndex = findProgramIndex(tokens[1]);
    if (programIndex < 0)
    {
        logError("program not found");
        return false;
    }

    MotorV2Program &program = s_programs[programIndex];
    String out = "[MOTOR2] Program ";
    out += program.name;
    out += " loops=";
    out += String(program.defaultLoops);
    out += " steps=";
    out += String(program.stepCount);
    for (uint8_t i = 0; i < program.stepCount; i++)
    {
        out += "\n  ";
        out += String(i);
        out += ": ";
        appendStepText(out, program.steps[i]);
    }
    LOG(out);
    return true;
}

static bool handleList()
{
    String out = "[MOTOR2] Programs:";
    bool any = false;
    for (uint8_t i = 0; i < CFG_MOTOR_V2_MAX_PROGRAMS; i++)
    {
        if (!s_programs[i].used)
            continue;
        any = true;
        out += "\n  ";
        out += s_programs[i].name;
        out += " steps=";
        out += String(s_programs[i].stepCount);
        out += " loops=";
        out += String(s_programs[i].defaultLoops);
    }
    if (!any)
        out += " none";
    LOG(out);
    return true;
}

static bool handleCenter(const String &body)
{
    String tokens[CFG_MOTOR_V2_MAX_TOKENS];
    const int count = splitTokens(body, tokens, CFG_MOTOR_V2_MAX_TOKENS);

    if (s_runner.active)
        stopProgram(false, "center command");

    uint32_t durationMs = 0;
    if (count >= 3)
    {
        long rawMs = 0;
        String err;
        if (!parseLongStrict(tokens[2], rawMs) ||
            !MotorV2Parameters::validateMoveDuration(rawMs, durationMs, err))
        {
            logError(err.length() ? err : "invalid center duration");
            return false;
        }
    }

    if (count == 1 || lowerCopy(tokens[1]) == "all")
    {
        for (uint8_t ch = 0; ch < CFG_SERVO_CHANNELS; ch++)
            servos_set_timed(ch, 90, durationMs);
        LOG("[MOTOR2] All channels centered");
        markMotorStateDirty();
        return true;
    }

    long rawChannel = 0;
    uint8_t channel = 0;
    String err;
    if (!parseLongStrict(tokens[1], rawChannel) ||
        !MotorV2Parameters::validateChannel(rawChannel, channel, err))
    {
        logError(err.length() ? err : "invalid center channel");
        return false;
    }
    servos_set_timed(channel, 90, durationMs);
    LOG("[MOTOR2] ch" + String(channel) + " centered");
    markMotorStateDirty();
    return true;
}

static bool handleFree(const String &body)
{
    String err;
    MotorV2Step step;
    if (!parseStepStatement(body, step, err))
    {
        logError(err);
        return false;
    }
    return executeDirectStep(step, "free");
}

static bool handleStop(const String &body)
{
    const bool freeOutputs =
        body.indexOf("free") >= 0 || startsWithIgnoreCase(body, "estop");
    stopProgram(freeOutputs, freeOutputs ? "emergency free" : "manual stop");
    LOG(freeOutputs ? "[MOTOR2] Stop/free applied" : "[MOTOR2] Stop/hold applied");
    return true;
}

static bool handleParams()
{
    String payload = "{\"type\":\"motor_v2_limits\",";
    MotorV2Parameters::appendLimitsJson(payload);
    payload += "}";
    LOG("[MOTOR2] " + payload);
    mqtt_publish(CFG_MQTT_TOPIC_MOTOR_STATE, payload);
    return true;
}

static bool handleStatus()
{
    publishMotorState();
    LOG("[MOTOR2] Status pca=" + String(servos_found() ? 1 : 0) +
        " running=" + String(s_runner.active ? 1 : 0) +
        " program=" + String(activeProgramName()));
    return true;
}

static bool handleStream(const String &body)
{
    String tokens[CFG_MOTOR_V2_MAX_TOKENS];
    const int count = splitTokens(body, tokens, CFG_MOTOR_V2_MAX_TOKENS);
    if (count < 2)
    {
        LOG("[MOTOR2] Stream status=" +
            String(statusStreamActive() ? "on" : "off") +
            " lease_ms=" + String(CFG_MOTOR_V2_STATUS_STREAM_LEASE_MS));
        return true;
    }

    String target = lowerCopy(tokens[1]);
    if (target == "off" || target == "quiet")
    {
        s_statusStreamUntilMs = 0;
        LOG("[MOTOR2] Status stream OFF");
        return true;
    }
    if (target != "status" && target != "state" && target != "all")
    {
        logError("stream syntax: stream status [on|off|once]");
        return false;
    }

    String action = count >= 3 ? lowerCopy(tokens[2]) : "once";
    if (action == "on" || action == "start" || action == "enable")
    {
        s_statusStreamUntilMs = millis() + CFG_MOTOR_V2_STATUS_STREAM_LEASE_MS;
        LOG("[MOTOR2] Status stream ON lease_ms=" +
            String(CFG_MOTOR_V2_STATUS_STREAM_LEASE_MS));
        publishMotorState();
        return true;
    }
    if (action == "off" || action == "stop" || action == "disable" || action == "quiet")
    {
        s_statusStreamUntilMs = 0;
        LOG("[MOTOR2] Status stream OFF");
        return true;
    }
    if (action == "once" || action == "now" || action == "snapshot")
    {
        publishMotorState();
        LOG("[MOTOR2] Status stream snapshot published");
        return true;
    }

    logError("stream syntax: stream status [on|off|once]");
    return false;
}

static bool handleHelp()
{
    LOG("[MOTOR2] Commands: motor:move <ch> <angle> [ms] | motor:pose <ms> <ch=angle>... | motor:load <name> [loops=N]; <steps>; [run] | motor:add <name> <step> | motor:set <name> <index> <step> | motor:override <name> [key=value...] | motor:run <name> [loops=N] | motor:stop [free] | motor:free [all|ch] | motor:center [all|ch] [ms] | motor:list | motor:show <name> | motor:params | motor:stream status [on|off|once]");
    return true;
}

bool motor_v2_handle_command(const String &payload)
{
    String body = stripMotorPrefix(payload);
    body.trim();
    if (startsWithIgnoreCase(body, "seq:"))
    {
        body = body.substring(4);
        body.replace(':', ' ');
        body.trim();
    }
    else if (startsWithIgnoreCase(body, "sequence:"))
    {
        body = body.substring(9);
        body.replace(':', ' ');
        body.trim();
    }

    if (body.length() == 0)
        return handleHelp();

    const String commandToken = firstToken(body);
    if (commandToken.length() == 0)
    {
        logError("invalid command");
        return false;
    }

    const String verb = lowerCopy(commandToken);

    if (verb == "help")
        return handleHelp();
    if (verb == "status")
        return handleStatus();
    if (verb == "params" || verb == "limits")
        return handleParams();
    if (verb == "stream")
        return handleStream(body);
    if (verb == "move" || verb == "pose" || verb == "wait")
        return parseAndRunDirectStep(body);
    if (verb == "free")
        return handleFree(body);
    if (verb == "center")
        return handleCenter(body);
    if (verb == "stop" || verb == "estop")
        return handleStop(body);
    if (verb == "load")
        return handleLoad(body);
    if (verb == "add")
        return handleAdd(body);
    if (verb == "set")
        return handleSet(body);
    if (verb == "override")
        return handleOverride(body);
    if (verb == "swap" || verb == "sync")   // NEW COMMAND ROUTING
        return handleSwap(body);
    if (verb == "run")
        return handleRun(body);
    if (verb == "clear")
        return handleClear(body);
    if (verb == "list")
        return handleList();
    if (verb == "show")
        return handleShow(body);

    logError("unknown command '" + commandToken + "'");
    return false;
}

static void motorCmdHandler(const String &msg)
{
    motor_v2_handle_command(msg);
}

static void legacyServoCmdHandler(const String &msg)
{
    int first = msg.indexOf(':');
    int second = (first == -1) ? -1 : msg.indexOf(':', first + 1);
    if (first == -1)
    {
        handleHelp();
        return;
    }

    String action = msg.substring(first + 1, second == -1 ? (int)msg.length() : second);
    action.toLowerCase();

    if (action == "status")
    {
        handleStatus();
        return;
    }
    if (action == "stream")
    {
        String translated = "stream";
        if (second != -1)
        {
            String rest = msg.substring(second + 1);
            rest.replace(':', ' ');
            translated += " " + rest;
        }
        handleStream(translated);
        return;
    }
    if (action == "free")
    {
        String translated = "free";
        if (second != -1)
            translated += " " + msg.substring(second + 1);
        handleFree(translated);
        return;
    }
    if (action == "center")
    {
        String translated = "center";
        if (second != -1)
            translated += " " + msg.substring(second + 1);
        handleCenter(translated);
        return;
    }

    if (second == -1)
    {
        logError("legacy syntax: servo:<ch>:<angle>[:ms]");
        return;
    }

    int third = msg.indexOf(':', second + 1);
    String translated = "move ";
    translated += msg.substring(first + 1, second);
    translated += " ";
    translated += msg.substring(second + 1, third == -1 ? (int)msg.length() : third);
    if (third != -1)
    {
        translated += " ";
        translated += msg.substring(third + 1);
    }
    motor_v2_handle_command(translated);
}

static int jsonValueStart(const String &payload, const char *key)
{
    String token = "\"" + String(key) + "\"";
    int keyPos = payload.indexOf(token);
    if (keyPos == -1)
        return -1;

    int colon = payload.indexOf(':', keyPos + token.length());
    if (colon == -1)
        return -1;

    int start = colon + 1;
    while (start < (int)payload.length() && isspace((unsigned char)payload[start]))
        start++;
    return start;
}

static bool jsonGetString(const String &payload, const char *key, String &out)
{
    int start = jsonValueStart(payload, key);
    if (start == -1 || start >= (int)payload.length() || payload[start] != '"')
        return false;

    int end = payload.indexOf('"', start + 1);
    if (end == -1)
        return false;

    out = payload.substring(start + 1, end);
    return true;
}

static bool jsonGetInt(const String &payload, const char *key, long &out)
{
    int start = jsonValueStart(payload, key);
    if (start == -1)
        return false;

    int end = start;
    if (end < (int)payload.length() && payload[end] == '-')
        end++;
    while (end < (int)payload.length() && isdigit((unsigned char)payload[end]))
        end++;
    if (end <= start)
        return false;

    out = payload.substring(start, end).toInt();
    return true;
}

static int jsonParseAngles(const String &payload, int *angles, int maxAngles)
{
    const String token = "\"angles\"";
    int keyPos = payload.indexOf(token);
    if (keyPos == -1)
        return 0;

    int start = payload.indexOf('[', keyPos + token.length());
    int end = payload.indexOf(']', start + 1);
    if (start == -1 || end == -1 || end <= start)
        return -1;

    int count = 0;
    int index = start + 1;
    while (index < end && count < maxAngles)
    {
        while (index < end &&
               (isspace((unsigned char)payload[index]) || payload[index] == ','))
            index++;
        if (index >= end)
            break;

        int valueStart = index;
        if (payload[index] == '-')
            index++;
        if (index >= end || !isdigit((unsigned char)payload[index]))
            return -1;
        while (index < end && isdigit((unsigned char)payload[index]))
            index++;

        angles[count++] = payload.substring(valueStart, index).toInt();

        int next = index;
        while (next < end && isspace((unsigned char)payload[next]))
            next++;
        if (next < end && payload[next] != ',')
            return -1;
    }
    return count;
}

static bool adoptSession(const String &session, const char *source)
{
    if (session.isEmpty())
    {
        logError(String("ignored ") + source + " packet with empty session");
        return false;
    }

    if (s_activeSession != session)
    {
        s_activeSession = session;
        s_lastMotionSeq = 0;
        s_lastHeartbeatSeq = 0;
        s_watchdogExpired = false;
        LOG("[MOTOR2] Control session -> " + s_activeSession);
    }
    return true;
}

void motor_v2_handle_motion_json(const String &payload)
{
    String packetType;
    jsonGetString(payload, "type", packetType);
    if (packetType == "gait_override")
    {
        String script;
        if (jsonGetString(payload, "script", script))
            motor_v2_handle_command(script);
        else
            publishMotorEvent("gait_override", "motion override received");
        return;
    }

    String script;
    if (jsonGetString(payload, "script", script))
    {
        motor_v2_handle_command(script);
        return;
    }

    if (!servos_found())
    {
        logError("PCA9685 not found - motion packet ignored");
        return;
    }

    String session;
    jsonGetString(payload, "session", session);
    if (!adoptSession(session, "motion"))
        return;

    long seq = 0;
    if (!jsonGetInt(payload, "seq", seq))
    {
        logError("ignored motion packet with no seq");
        return;
    }
    if ((uint32_t)seq <= s_lastMotionSeq)
    {
        publishMotorEvent("stale_motion", "rejected old seq");
        return;
    }

    long channel = -1;
    long angle = -1;
    long durationMs = 0;
    int angles[CFG_SERVO_CHANNELS];
    const int angleCount = jsonParseAngles(payload, angles, CFG_SERVO_CHANNELS);
    const bool hasChannel = jsonGetInt(payload, "channel", channel);
    const bool hasAngle = jsonGetInt(payload, "angle", angle);
    jsonGetInt(payload, "duration_ms", durationMs);

    if (angleCount < 0)
    {
        logError("malformed motion angles");
        return;
    }

    if (s_runner.active)
    {
        publishMotorEvent(
            "motion_intercepted",
            "direct motion ignored while RAM program is running");
        return;
    }

    String command;
    if (angleCount > 0)
    {
        command = "pose ";
        command += String(max(0L, durationMs));
        for (int i = 0; i < angleCount; i++)
        {
            command += " ";
            command += String(i);
            command += "=";
            command += String(angles[i]);
        }
    }
    else if (hasChannel && hasAngle)
    {
        command = "move ";
        command += String(channel);
        command += " ";
        command += String(angle);
        command += " ";
        command += String(max(0L, durationMs));
    }
    else
    {
        logError("motion packet has no angles");
        return;
    }

    if (!motor_v2_handle_command(command))
        return;

    s_lastMotionSeq = (uint32_t)seq;
    s_lastHeartbeatMs = millis();
    s_watchdogExpired = false;
    s_watchdogArmed = true;
    markMotorStateDirty();
}

void motor_v2_handle_heartbeat_json(const String &payload)
{
    String session;
    jsonGetString(payload, "session", session);
    if (!adoptSession(session, "heartbeat"))
        return;

    long seq = 0;
    if (jsonGetInt(payload, "seq", seq) && (uint32_t)seq > s_lastHeartbeatSeq)
        s_lastHeartbeatSeq = (uint32_t)seq;

    s_lastHeartbeatMs = millis();
    s_watchdogExpired = false;
    markMotorStateDirty();
}

void motor_v2_init()
{
    for (uint8_t i = 0; i < CFG_MOTOR_V2_MAX_PROGRAMS; i++)
        resetProgram(s_programs[i]);

    s_runner = {};
    s_runner.pendingProgramIndex = -1; // NEW: Reset pending swap
    s_lastMotorStateMs = millis();
    s_statusStreamUntilMs = 0;
    s_motorStateDirty = false;

    servos_init();
    LOG("[MOTOR2] Ready - RAM sequence layer active");
}

void motor_v2_register_commands()
{
    cmd_register("motor:", motorCmdHandler);
    cmd_register("motor2:", motorCmdHandler);
    cmd_register("servo:", legacyServoCmdHandler);
    LOG("[MOTOR2] Registered motor:, motor2:, and legacy servo: commands");
}

void motor_v2_handle()
{
    const uint32_t now = millis();

    servos_handle();

    if (s_watchdogArmed && !s_activeSession.isEmpty() && !s_watchdogExpired &&
        (now - s_lastHeartbeatMs) > CFG_CONTROL_HEARTBEAT_TIMEOUT_MS)
    {
        stopProgram(true, "heartbeat lost");
        s_watchdogExpired = true;
        LOG("[MOTOR2] Watchdog timeout - all channels freed");
    }

    if (s_runner.active)
    {
        if (!servos_found())
            stopProgram(true, "PCA9685 lost");
        else
            advanceRunner(now);
    }

    if (statusStreamActive(now) &&
        (s_motorStateDirty || (now - s_lastMotorStateMs) > CFG_CONTROL_STATE_PUBLISH_MS))
        publishMotorState();
}
