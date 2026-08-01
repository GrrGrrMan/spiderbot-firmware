#include "servos.h"
#include "Build/config/target_config.h"
#include "Build/Log/logger.h"
#include "Build/Log/cmd_registry.h"
#include "Network/mqtt/mqtt_trigger.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

#ifndef CFG_PCA9685_BOARD_COUNT
#define CFG_PCA9685_BOARD_COUNT 1
#endif

#ifndef CFG_PCA9685_SECONDARY_ADDR
#define CFG_PCA9685_SECONDARY_ADDR 0x41
#endif

#ifndef CFG_SERVO_SECONDARY_CHANNEL_BASE
#define CFG_SERVO_SECONDARY_CHANNEL_BASE 16
#endif

#ifndef CFG_I2C_FREQ_HZ
#define CFG_I2C_FREQ_HZ 100000UL
#endif

static Adafruit_PWMServoDriver pwmPrimary = Adafruit_PWMServoDriver(CFG_PCA9685_ADDR);
#if CFG_PCA9685_BOARD_COUNT >= 2
static Adafruit_PWMServoDriver pwmSecondary = Adafruit_PWMServoDriver(CFG_PCA9685_SECONDARY_ADDR);
#endif

static bool pca9685Found[CFG_PCA9685_BOARD_COUNT] = {false};
static int s_angles[CFG_SERVO_CHANNELS];
static int s_targets[CFG_SERVO_CHANNELS];
static float s_motionStartAngles[CFG_SERVO_CHANNELS];
static uint32_t s_motionStartMs[CFG_SERVO_CHANNELS];
static uint32_t s_motionDurationMs[CFG_SERVO_CHANNELS];
static bool s_motionActive[CFG_SERVO_CHANNELS];

static String s_activeSession;
static uint32_t s_lastMotionSeq = 0;
static uint32_t s_lastHeartbeatSeq = 0;
static uint32_t s_lastHeartbeatMs = 0;
static uint32_t s_lastStatePublishMs = 0;
static uint32_t s_lastProbeMs = 0;
static uint32_t s_lastMotionTickMs = 0;
static bool s_watchdogExpired = false;
static bool s_watchdogArmed = false;
static bool s_stateDirty = false;

static Adafruit_PWMServoDriver *driverForBoard(uint8_t board)
{
    if (board == 0)
        return &pwmPrimary;
#if CFG_PCA9685_BOARD_COUNT >= 2
    if (board == 1)
        return &pwmSecondary;
#endif
    return nullptr;
}

static bool mapServoChannel(uint8_t channel, uint8_t &board, uint8_t &localChannel)
{
    if (channel >= CFG_SERVO_CHANNELS)
        return false;

#if CFG_PCA9685_BOARD_COUNT >= 2
    if (channel >= CFG_SERVO_SECONDARY_CHANNEL_BASE)
    {
        board = 1;
        localChannel = channel - CFG_SERVO_SECONDARY_CHANNEL_BASE;
    }
    else
    {
        board = 0;
        localChannel = channel;
    }
#else
    board = 0;
    localChannel = channel;
#endif

    return board < CFG_PCA9685_BOARD_COUNT && localChannel < 16;
}

static bool anyPca9685Found()
{
    for (uint8_t i = 0; i < CFG_PCA9685_BOARD_COUNT; i++)
    {
        if (pca9685Found[i])
            return true;
    }
    return false;
}

static void markStateDirty()
{
    s_stateDirty = true;
}

static bool setPwmForAngle(uint8_t channel, int angle)
{
    uint8_t board = 0;
    uint8_t localChannel = 0;
    if (!mapServoChannel(channel, board, localChannel) || !pca9685Found[board])
        return false;

    Adafruit_PWMServoDriver *driver = driverForBoard(board);
    if (!driver)
        return false;

    int pulse = map(constrain(angle, 0, 180), 0, 180, CFG_SERVO_MIN, CFG_SERVO_MAX);
    driver->setPWM(localChannel, 0, pulse);
    return true;
}

static bool setFullOff(uint8_t channel)
{
    uint8_t board = 0;
    uint8_t localChannel = 0;
    if (!mapServoChannel(channel, board, localChannel) || !pca9685Found[board])
        return false;

    Adafruit_PWMServoDriver *driver = driverForBoard(board);
    if (!driver)
        return false;

    driver->setPWM(localChannel, 0, 4096);
    return true;
}

static float getChannelAngleNow(uint8_t channel, uint32_t now)
{
    if (channel >= CFG_SERVO_CHANNELS)
        return 0.0f;

    if (!s_motionActive[channel] || s_motionDurationMs[channel] == 0)
        return (s_angles[channel] < 0) ? 0.0f : (float)s_angles[channel];

    const uint32_t elapsed = now - s_motionStartMs[channel];
    if (elapsed >= s_motionDurationMs[channel])
        return (float)s_targets[channel];

    const float progress = (float)elapsed / (float)s_motionDurationMs[channel];
    return s_motionStartAngles[channel] +
           ((float)s_targets[channel] - s_motionStartAngles[channel]) * progress;
}

static void stopMotion(uint8_t channel)
{
    if (channel >= CFG_SERVO_CHANNELS)
        return;

    s_motionActive[channel] = false;
    s_motionDurationMs[channel] = 0;
    s_motionStartMs[channel] = 0;
}

static uint32_t resolveMoveDurationMs(int fromAngle, int toAngle, uint32_t requestedMs)
{
    const int distance = abs(toAngle - fromAngle);
    uint32_t speedLimitedMs = 0;

    if (CFG_SERVO_MAX_SPEED_DPS > 0.0f && distance > 0)
    {
        speedLimitedMs = (uint32_t)ceilf(
            ((float)distance * 1000.0f) / CFG_SERVO_MAX_SPEED_DPS);
    }

    if (requestedMs == 0)
        return speedLimitedMs;
    if (speedLimitedMs == 0)
        return requestedMs;
    return max(requestedMs, speedLimitedMs);
}

static void applyAngleImmediate(uint8_t channel, int angle)
{
    angle = constrain(angle, 0, 180);
    setPwmForAngle(channel, angle);
    s_angles[channel] = angle;
    s_targets[channel] = angle;
    stopMotion(channel);
}

static bool probePca9685(uint8_t board)
{
    const uint8_t address =
        board == 0 ? CFG_PCA9685_ADDR : CFG_PCA9685_SECONDARY_ADDR;
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

static void syncOutputsAfterAttach(uint8_t board)
{
    bool restoredAny = false;
    for (int i = 0; i < CFG_SERVO_CHANNELS; i++)
    {
        uint8_t mappedBoard = 0;
        uint8_t localChannel = 0;
        if (!mapServoChannel((uint8_t)i, mappedBoard, localChannel) ||
            mappedBoard != board)
        {
            continue;
        }

        if (s_angles[i] >= 0)
        {
            setPwmForAngle((uint8_t)i, s_angles[i]);
            s_targets[i] = s_angles[i];
            stopMotion((uint8_t)i);
            restoredAny = true;
        }
        else
        {
            setFullOff((uint8_t)i);
            s_targets[i] = -1;
            stopMotion((uint8_t)i);
        }
    }

    // if (!restoredAny)
    //     servos_center_all();
}

static bool attachPca9685(uint8_t board, bool initialBoot)
{
    if (board >= CFG_PCA9685_BOARD_COUNT || !probePca9685(board))
        return false;

    Adafruit_PWMServoDriver *driver = driverForBoard(board);
    if (!driver)
        return false;

    driver->begin();
    driver->setPWMFreq(CFG_SERVO_FREQ_HZ);

    Wire.setClock(CFG_I2C_FREQ_HZ);

    pca9685Found[board] = true;
    syncOutputsAfterAttach(board);
    markStateDirty();

    const uint8_t address =
        board == 0 ? CFG_PCA9685_ADDR : CFG_PCA9685_SECONDARY_ADDR;
    if (initialBoot)
        LOG("[SERVO] PCA9685 board " + String(board) + " found at 0x" +
            String(address, HEX));
    else
        LOG("[SERVO] PCA9685 board " + String(board) + " found at 0x" +
            String(address, HEX) + " - runtime attach");

    return true;
}

static String jsonGetString(const String &payload, const char *key)
{
    String token = "\"" + String(key) + "\"";
    int keyPos = payload.indexOf(token);
    if (keyPos == -1)
        return "";

    int colon = payload.indexOf(':', keyPos + token.length());
    if (colon == -1)
        return "";

    int firstQuote = payload.indexOf('"', colon + 1);
    if (firstQuote == -1)
        return "";

    int secondQuote = payload.indexOf('"', firstQuote + 1);
    if (secondQuote == -1)
        return "";

    return payload.substring(firstQuote + 1, secondQuote);
}

static bool jsonGetInt(const String &payload, const char *key, long &out)
{
    String token = "\"" + String(key) + "\"";
    int keyPos = payload.indexOf(token);
    if (keyPos == -1)
        return false;

    int colon = payload.indexOf(':', keyPos + token.length());
    if (colon == -1)
        return false;

    int start = colon + 1;
    while (start < (int)payload.length() && isspace(payload[start]))
        start++;

    int end = start;
    if (end < (int)payload.length() && payload[end] == '-')
        end++;
    while (end < (int)payload.length() && isdigit(payload[end]))
        end++;

    if (end <= start)
        return false;

    out = payload.substring(start, end).toInt();
    return true;
}

static int jsonParseAngles(const String &payload, int *angles, int maxAngles)
{
    String token = "\"angles\"";
    int keyPos = payload.indexOf(token);
    if (keyPos == -1)
        return 0;

    int start = payload.indexOf('[', keyPos + token.length());
    int end = payload.indexOf(']', start + 1);
    if (start == -1 || end == -1 || end <= start)
        return 0;

    String body = payload.substring(start + 1, end);
    int count = 0;
    int index = 0;

    while (index < (int)body.length() && count < maxAngles)
    {
        while (index < (int)body.length() && (isspace(body[index]) || body[index] == ','))
            index++;
        if (index >= (int)body.length())
            break;

        const bool negative = body[index] == '-';
        int valueStart = index;
        if (negative)
            index++;

        if (index >= (int)body.length() || !isdigit(body[index]))
            return -1;

        while (index < (int)body.length() && isdigit(body[index]))
            index++;

        int next = index;
        while (next < (int)body.length() && isspace(body[next]))
            next++;
        if (next < (int)body.length() && body[next] != ',')
            return -1;

        angles[count++] = body.substring(valueStart, index).toInt();
    }

    return count;
}

static bool adoptSession(const String &session, const char *source)
{
    if (session.isEmpty())
    {
        LOG(String("[SERVO] Ignored ") + source + " packet with empty session");
        return false;
    }

    if (s_activeSession != session)
    {
        s_activeSession = session;
        s_lastMotionSeq = 0;
        s_lastHeartbeatSeq = 0;
        s_watchdogExpired = false;
        LOG("[SERVO] Control session -> " + s_activeSession);
    }
    return true;
}

static void publishEvent(const String &type, const String &detail)
{
    String out = "{\"type\":\"" + type + "\",\"session\":\"" + s_activeSession +
                 "\",\"detail\":\"" + detail + "\"}";
    mqtt_publish(CFG_MQTT_TOPIC_EVENT, out);
}

static void publishState()
{
    String out = "{\"session\":\"" + s_activeSession + "\",\"watchdog\":" +
                 String(s_watchdogExpired ? 1 : 0) + ",\"angles\":[";
    for (int i = 0; i < CFG_SERVO_CHANNELS; i++)
    {
        out += String(s_angles[i]);
        if (i < CFG_SERVO_CHANNELS - 1)
            out += ',';
    }
    out += "]}";

    mqtt_publish(CFG_MQTT_TOPIC_STATE, out);
    s_lastStatePublishMs = millis();
    s_stateDirty = false;
}

static uint32_t resolvePoseDurationMs(const int *angles, int count, uint32_t requestedMs)
{
    const int limit = min(count, CFG_SERVO_CHANNELS);
    uint32_t resolvedMs = requestedMs;
    const uint32_t now = millis();

    for (int i = 0; i < limit; i++)
    {
        if (s_angles[i] < 0)
            continue;

        const int currentAngle = (int)lroundf(getChannelAngleNow((uint8_t)i, now));
        resolvedMs = max(
            resolvedMs,
            resolveMoveDurationMs(currentAngle, constrain(angles[i], 0, 180), 0));
    }

    return resolvedMs;
}

static void applyPoseFrame(const int *angles, int count, uint32_t requestedMs)
{
    const int limit = min(count, CFG_SERVO_CHANNELS);
    const uint32_t durationMs = resolvePoseDurationMs(angles, count, requestedMs);

    for (int i = 0; i < limit; i++)
        servos_set_timed((uint8_t)i, angles[i], durationMs);
}

static void updateMotionOutputs(uint32_t now)
{
    for (uint8_t channel = 0; channel < CFG_SERVO_CHANNELS; channel++)
    {
        if (!s_motionActive[channel] || s_angles[channel] < 0)
            continue;

        const float nextAngle = getChannelAngleNow(channel, now);
        const int roundedAngle = constrain((int)lroundf(nextAngle), 0, 180);
        const bool finished =
            (now - s_motionStartMs[channel]) >= s_motionDurationMs[channel];

        if (roundedAngle != s_angles[channel])
            setPwmForAngle(channel, roundedAngle);

        s_angles[channel] = roundedAngle;

        if (finished)
        {
            s_angles[channel] = s_targets[channel];
            setPwmForAngle(channel, s_angles[channel]);
            stopMotion(channel);
            markStateDirty();
        }
    }
}

void servos_init()
{
    for (int i = 0; i < CFG_SERVO_CHANNELS; i++)
    {
        s_angles[i] = -1;
        s_targets[i] = -1;
        s_motionStartAngles[i] = 0.0f;
        s_motionStartMs[i] = 0;
        s_motionDurationMs[i] = 0;
        s_motionActive[i] = false;
    }

    Wire.begin(CFG_I2C_SDA, CFG_I2C_SCL);
    bool foundAny = false;
    for (uint8_t board = 0; board < CFG_PCA9685_BOARD_COUNT; board++)
    {
        if (attachPca9685(board, true))
            foundAny = true;
    }
    if (!foundAny)
    {
        LOG("[SERVO] No PCA9685 boards found - check wiring");
    }

    markStateDirty();
    s_lastMotionTickMs = millis();
    LOG("[SERVO] Ready");
}

void servos_handle()
{
    const uint32_t now = millis();

    if (!anyPca9685Found())
    {
        if ((now - s_lastProbeMs) > CFG_SERVO_REPROBE_MS)
        {
            s_lastProbeMs = now;
            for (uint8_t board = 0; board < CFG_PCA9685_BOARD_COUNT; board++)
            {
                if (!pca9685Found[board])
                    attachPca9685(board, false);
            }
        }
        return;
    }

    if (s_watchdogArmed && !s_activeSession.isEmpty() && !s_watchdogExpired &&
        (now - s_lastHeartbeatMs) > CFG_CONTROL_HEARTBEAT_TIMEOUT_MS)
    {
        servos_free_all();
        s_watchdogExpired = true;
        s_watchdogArmed = false;
        publishEvent("watchdog_timeout", "heartbeat lost");
        LOG("[SERVO] Watchdog timeout - all channels freed");
    }

    if ((now - s_lastMotionTickMs) >= CFG_SERVO_MOTION_TICK_MS)
    {
        s_lastMotionTickMs = now;
        updateMotionOutputs(now);
    }

#if !defined(FEATURE_MOTOR_V2)
    if (s_stateDirty || (now - s_lastStatePublishMs) > CFG_CONTROL_STATE_PUBLISH_MS)
        publishState();
#endif

    if ((now - s_lastProbeMs) > CFG_SERVO_REPROBE_MS)
    {
        s_lastProbeMs = now;
        for (uint8_t board = 0; board < CFG_PCA9685_BOARD_COUNT; board++)
        {
            if (!pca9685Found[board])
                attachPca9685(board, false);
        }
    }
}

bool servos_found() { return anyPca9685Found(); }

bool pca9685_set_full_off(uint8_t channel)
{
    if (channel >= CFG_SERVO_CHANNELS)
        return false;

    return setFullOff(channel);
}

bool pca9685_set_pwm_duty(uint8_t channel, uint16_t duty12)
{
    if (channel >= CFG_SERVO_CHANNELS)
        return false;

    if (duty12 == 0)
        return setFullOff(channel);
    else
    {
        uint8_t board = 0;
        uint8_t localChannel = 0;
        if (!mapServoChannel(channel, board, localChannel) || !pca9685Found[board])
            return false;

        Adafruit_PWMServoDriver *driver = driverForBoard(board);
        if (!driver)
            return false;

        driver->setPWM(localChannel, 0, duty12 > 4095 ? 4095 : duty12);
    }

    return true;
}

void servos_set(uint8_t channel, int angle)
{
    servos_set_timed(channel, angle, 0);
}

void servos_set_timed(uint8_t channel, int angle, uint32_t durationMs)
{
    if (channel >= CFG_SERVO_CHANNELS)
        return;

    angle = constrain(angle, 0, 180);
    const uint32_t now = millis();

    if (s_angles[channel] < 0)
    {
        applyAngleImmediate(channel, angle);
        markStateDirty();
        return;
    }

    const int startAngle = (int)lroundf(getChannelAngleNow(channel, now));
    const uint32_t resolvedMs = resolveMoveDurationMs(startAngle, angle, durationMs);

    if (startAngle == angle || resolvedMs == 0)
    {
        applyAngleImmediate(channel, angle);
        markStateDirty();
        return;
    }

    s_motionStartAngles[channel] = (float)startAngle;
    s_motionStartMs[channel] = now;
    s_motionDurationMs[channel] = resolvedMs;
    s_targets[channel] = angle;
    s_motionActive[channel] = true;
    s_angles[channel] = startAngle;
    markStateDirty();
}

void servos_center_all()
{
    for (int i = 0; i < CFG_SERVO_CHANNELS; i++)
        servos_set((uint8_t)i, 90);
}

void servos_free(uint8_t channel)
{
    if (channel >= CFG_SERVO_CHANNELS)
        return;

    setFullOff(channel);
    stopMotion(channel);
    s_angles[channel] = -1;
    s_targets[channel] = -1;
    markStateDirty();
    LOG("[SERVO] ch" + String(channel) + " freed");
}

void servos_free_all()
{
    for (uint8_t i = 0; i < CFG_SERVO_CHANNELS; i++)
    {
        setFullOff(i);
        stopMotion(i);
        s_angles[i] = -1;
        s_targets[i] = -1;
    }
    s_watchdogArmed = false;
    markStateDirty();
    LOG("[SERVO] All channels freed");
}

int servos_get_angle(uint8_t channel)
{
    if (channel >= CFG_SERVO_CHANNELS)
        return -1;
    return s_angles[channel];
}

void servos_handle_motion_json(const String &payload)
{
    if (!servos_found())
    {
        LOG("[SERVO] PCA9685 not found - motion packet ignored");
        return;
    }

    String session = jsonGetString(payload, "session");
    if (!adoptSession(session, "motion"))
        return;

    long seq = 0;
    if (!jsonGetInt(payload, "seq", seq))
    {
        LOG("[SERVO] Ignored motion packet with no seq");
        return;
    }

    if ((uint32_t)seq <= s_lastMotionSeq)
    {
        publishEvent("stale_motion", "rejected old seq");
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
        LOG("[SERVO] Ignored malformed motion angles");
        return;
    }

    if (angleCount > 0)
    {
        applyPoseFrame(angles, angleCount, (uint32_t)max(0L, durationMs));
        if (durationMs > 0)
            LOG("[SERVO] Applied pose frame seq " + String(seq) + " over " + String(durationMs) + " ms");
        else
            LOG("[SERVO] Applied pose frame seq " + String(seq));
    }
    else if (hasChannel && hasAngle)
    {
        servos_set_timed((uint8_t)channel, (int)angle, (uint32_t)max(0L, durationMs));
        if (durationMs > 0)
            LOG("[SERVO] ch" + String(channel) + " -> " + String(angle) + " deg in " + String(durationMs) + " ms via motion seq " + String(seq));
        else
            LOG("[SERVO] ch" + String(channel) + " -> " + String(angle) + " deg via motion seq " + String(seq));
    }
    else
    {
        LOG("[SERVO] Ignored motion packet with no angles");
        return;
    }

    s_lastMotionSeq = (uint32_t)seq;
    s_lastHeartbeatMs = millis();
    s_watchdogExpired = false;
    s_watchdogArmed = true;
    markStateDirty();
}

void servos_handle_heartbeat_json(const String &payload)
{
    String session = jsonGetString(payload, "session");
    if (!adoptSession(session, "heartbeat"))
        return;

    long seq = 0;
    if (jsonGetInt(payload, "seq", seq) && (uint32_t)seq > s_lastHeartbeatSeq)
        s_lastHeartbeatSeq = (uint32_t)seq;

    s_lastHeartbeatMs = millis();
    if (s_watchdogExpired)
        markStateDirty();
    s_watchdogExpired = false;
}

static void servoCmdHandler(const String &msg)
{
    if (!servos_found())
    {
        LOG("[SERVO] PCA9685 not found - command ignored");
        return;
    }

    int first = msg.indexOf(':');
    int second = (first == -1) ? -1 : msg.indexOf(':', first + 1);

    if (first == -1)
    {
        LOG("[SERVO] Usage: servo:<ch>:<angle>[:<ms>] | servo:free[:<ch>] | servo:center[:<ch>] | servo:status");
        return;
    }

    String action = msg.substring(first + 1, second == -1 ? (int)msg.length() : second);

    if (action == "free")
    {
        if (second == -1)
            servos_free_all();
        else
            servos_free((uint8_t)msg.substring(second + 1).toInt());
        return;
    }

    if (action == "center")
    {
        if (second == -1)
        {
            servos_center_all();
            LOG("[SERVO] All channels centered at 90 deg");
        }
        else
        {
            uint8_t ch = (uint8_t)msg.substring(second + 1).toInt();
            servos_set(ch, 90);
            LOG("[SERVO] ch" + String(ch) + " centered at 90 deg");
        }
        return;
    }

    if (action == "status")
    {
        String out = "[SERVO] Status:\n";
        for (int i = 0; i < CFG_SERVO_CHANNELS; i++)
        {
            out += "  ch" + String(i) + ": ";
            out += (s_angles[i] < 0) ? String("free") : String(s_angles[i]) + " deg";
            if (i < CFG_SERVO_CHANNELS - 1)
                out += "\n";
        }
        LOG(out);
        return;
    }

    if (second == -1)
    {
        LOG("[SERVO] Usage: servo:<ch>:<angle>[:<ms>] | servo:free[:<ch>] | servo:center[:<ch>] | servo:status");
        return;
    }

    int third = msg.indexOf(':', second + 1);
    uint8_t ch = (uint8_t)msg.substring(first + 1, second).toInt();
    int angle = msg.substring(second + 1, third == -1 ? (int)msg.length() : third).toInt();
    uint32_t durationMs = 0;
    if (third != -1)
        durationMs = (uint32_t)max(0L, msg.substring(third + 1).toInt());

    servos_set_timed(ch, angle, durationMs);
    if (durationMs > 0)
        LOG("[SERVO] ch" + String(ch) + " -> " + String(angle) + " deg in " + String(durationMs) + " ms");
    else
        LOG("[SERVO] ch" + String(ch) + " -> " + String(angle) + " deg");
}

void servos_register_commands()
{
    cmd_register("servo:", servoCmdHandler);
}
