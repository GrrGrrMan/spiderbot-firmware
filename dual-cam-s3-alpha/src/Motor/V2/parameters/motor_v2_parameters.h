#pragma once

#include <Arduino.h>
#include "Build/config/target_config.h"

#ifndef CFG_MOTOR_V2_MAX_PROGRAMS
#define CFG_MOTOR_V2_MAX_PROGRAMS 4
#endif

#ifndef CFG_MOTOR_V2_MAX_STEPS
#define CFG_MOTOR_V2_MAX_STEPS 24
#endif

#ifndef CFG_MOTOR_V2_MAX_NAME_LEN
#define CFG_MOTOR_V2_MAX_NAME_LEN 15
#endif

#ifndef CFG_MOTOR_V2_MAX_TOKENS
#define CFG_MOTOR_V2_MAX_TOKENS 24
#endif

#ifndef CFG_MOTOR_V2_MIN_ANGLE
#define CFG_MOTOR_V2_MIN_ANGLE 0
#endif

#ifndef CFG_MOTOR_V2_MAX_ANGLE
#define CFG_MOTOR_V2_MAX_ANGLE 180
#endif

#ifndef CFG_MOTOR_V2_MAX_MOVE_MS
#define CFG_MOTOR_V2_MAX_MOVE_MS 120000UL
#endif

#ifndef CFG_MOTOR_V2_MAX_WAIT_MS
#define CFG_MOTOR_V2_MAX_WAIT_MS 300000UL
#endif

#ifndef CFG_MOTOR_V2_MAX_LOOPS
#define CFG_MOTOR_V2_MAX_LOOPS 10000UL
#endif

#ifndef CFG_MOTOR_V2_DEFAULT_MOVE_MS
#define CFG_MOTOR_V2_DEFAULT_MOVE_MS 0UL
#endif

#ifndef CFG_MOTOR_V2_ENFORCE_SPEED_LIMIT
#define CFG_MOTOR_V2_ENFORCE_SPEED_LIMIT 1
#endif

#ifndef CFG_MOTOR_V2_POSE_CHANNEL_STAGGER_MS
#define CFG_MOTOR_V2_POSE_CHANNEL_STAGGER_MS 8UL
#endif

namespace MotorV2Parameters
{
enum class StepKind : uint8_t
{
    Empty,
    Move,
    Pose,
    Wait,
    Free
};

const char *stepKindName(StepKind kind);

bool validateProgramName(const String &name, String &err);
bool validateChannel(long raw, uint8_t &out, String &err);
bool validateAngle(long raw, int16_t &out, String &err);
bool validateMoveDuration(long raw, uint32_t &out, String &err);
bool validateWaitDuration(long raw, uint32_t &out, String &err);
bool validateLoops(long raw, uint32_t &out, String &err);

uint32_t resolveMoveDurationMs(int fromAngle,
                               int toAngle,
                               uint32_t requestedMs,
                               bool *speedLimited = nullptr);

void appendLimitsJson(String &out);
} // namespace MotorV2Parameters
