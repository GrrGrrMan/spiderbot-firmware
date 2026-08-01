#include "motor_v2_parameters.h"

#include <ctype.h>
#include <math.h>

#ifndef CFG_PCA9685_BOARD_COUNT
#define CFG_PCA9685_BOARD_COUNT 1
#endif

#ifndef CFG_PCA9685_SECONDARY_ADDR
#define CFG_PCA9685_SECONDARY_ADDR 0
#endif

#ifndef CFG_SERVO_SECONDARY_CHANNEL_BASE
#define CFG_SERVO_SECONDARY_CHANNEL_BASE 16
#endif

namespace MotorV2Parameters
{
static void setErr(String &err, const String &value)
{
    err = value;
}

const char *stepKindName(StepKind kind)
{
    switch (kind)
    {
    case StepKind::Move:
        return "move";
    case StepKind::Pose:
        return "pose";
    case StepKind::Wait:
        return "wait";
    case StepKind::Free:
        return "free";
    case StepKind::Empty:
    default:
        return "empty";
    }
}

bool validateProgramName(const String &name, String &err)
{
    if (name.length() == 0)
    {
        setErr(err, "program name is required");
        return false;
    }
    if (name.length() > CFG_MOTOR_V2_MAX_NAME_LEN)
    {
        setErr(err, "program name too long");
        return false;
    }

    for (size_t i = 0; i < name.length(); i++)
    {
        const char ch = name[i];
        if (!(isalnum((unsigned char)ch) || ch == '_' || ch == '-'))
        {
            setErr(err, "program name may use A-Z, 0-9, _ and - only");
            return false;
        }
    }
    return true;
}

bool validateChannel(long raw, uint8_t &out, String &err)
{
    if (raw < 0 || raw >= CFG_SERVO_CHANNELS)
    {
        setErr(err, "channel out of range 0.." + String(CFG_SERVO_CHANNELS - 1));
        return false;
    }
    out = (uint8_t)raw;
    return true;
}

bool validateAngle(long raw, int16_t &out, String &err)
{
    if (raw < CFG_MOTOR_V2_MIN_ANGLE || raw > CFG_MOTOR_V2_MAX_ANGLE)
    {
        setErr(err, "angle out of range " + String(CFG_MOTOR_V2_MIN_ANGLE) +
                        ".." + String(CFG_MOTOR_V2_MAX_ANGLE));
        return false;
    }
    out = (int16_t)raw;
    return true;
}

static bool validateDuration(long raw,
                             uint32_t maxMs,
                             const char *label,
                             uint32_t &out,
                             String &err)
{
    if (raw < 0)
    {
        setErr(err, String(label) + " cannot be negative");
        return false;
    }
    if ((uint32_t)raw > maxMs)
    {
        setErr(err, String(label) + " exceeds " + String(maxMs) + " ms");
        return false;
    }
    out = (uint32_t)raw;
    return true;
}

bool validateMoveDuration(long raw, uint32_t &out, String &err)
{
    return validateDuration(raw, CFG_MOTOR_V2_MAX_MOVE_MS, "move duration", out, err);
}

bool validateWaitDuration(long raw, uint32_t &out, String &err)
{
    return validateDuration(raw, CFG_MOTOR_V2_MAX_WAIT_MS, "wait duration", out, err);
}

bool validateLoops(long raw, uint32_t &out, String &err)
{
    if (raw < 0)
    {
        setErr(err, "loop count cannot be negative");
        return false;
    }
    if ((uint32_t)raw > CFG_MOTOR_V2_MAX_LOOPS)
    {
        setErr(err, "loop count exceeds " + String(CFG_MOTOR_V2_MAX_LOOPS));
        return false;
    }
    out = (uint32_t)raw;
    return true;
}

uint32_t resolveMoveDurationMs(int fromAngle,
                               int toAngle,
                               uint32_t requestedMs,
                               bool *speedLimited)
{
    if (speedLimited)
        *speedLimited = false;

    const int distance = abs(toAngle - fromAngle);
    uint32_t speedLimitedMs = 0;

#if CFG_MOTOR_V2_ENFORCE_SPEED_LIMIT
    if (CFG_SERVO_MAX_SPEED_DPS > 0.0f && distance > 0)
    {
        speedLimitedMs = (uint32_t)ceilf(
            ((float)distance * 1000.0f) / CFG_SERVO_MAX_SPEED_DPS);
    }
#endif

    uint32_t resolved = requestedMs;
    if (resolved == 0)
        resolved = speedLimitedMs;
    else if (speedLimitedMs > resolved)
        resolved = speedLimitedMs;

    if (speedLimited && speedLimitedMs > requestedMs)
        *speedLimited = true;

    return resolved;
}

void appendLimitsJson(String &out)
{
    out += "\"channels\":";
    out += String(CFG_SERVO_CHANNELS);
    out += ",\"angle_min\":";
    out += String(CFG_MOTOR_V2_MIN_ANGLE);
    out += ",\"angle_max\":";
    out += String(CFG_MOTOR_V2_MAX_ANGLE);
    out += ",\"max_programs\":";
    out += String(CFG_MOTOR_V2_MAX_PROGRAMS);
    out += ",\"max_steps\":";
    out += String(CFG_MOTOR_V2_MAX_STEPS);
    out += ",\"max_move_ms\":";
    out += String(CFG_MOTOR_V2_MAX_MOVE_MS);
    out += ",\"max_wait_ms\":";
    out += String(CFG_MOTOR_V2_MAX_WAIT_MS);
    out += ",\"max_loops\":";
    out += String(CFG_MOTOR_V2_MAX_LOOPS);
    out += ",\"speed_limit_dps\":";
    out += String(CFG_SERVO_MAX_SPEED_DPS);
    out += ",\"speed_limit_enforced\":";
    out += String(CFG_MOTOR_V2_ENFORCE_SPEED_LIMIT ? 1 : 0);
    out += ",\"pose_channel_stagger_ms\":";
    out += String(CFG_MOTOR_V2_POSE_CHANNEL_STAGGER_MS);
    out += ",\"i2c_sda\":";
    out += String(CFG_I2C_SDA);
    out += ",\"i2c_scl\":";
    out += String(CFG_I2C_SCL);
    out += ",\"pca9685_addr\":";
    out += String(CFG_PCA9685_ADDR);
    out += ",\"pca9685_secondary_addr\":";
    out += String(CFG_PCA9685_SECONDARY_ADDR);
    out += ",\"pca9685_board_count\":";
    out += String(CFG_PCA9685_BOARD_COUNT);
    out += ",\"servo_secondary_channel_base\":";
    out += String(CFG_SERVO_SECONDARY_CHANNEL_BASE);
    out += ",\"pwm_freq_hz\":";
    out += String(CFG_SERVO_FREQ_HZ);
}
} // namespace MotorV2Parameters
