#pragma once

#include <Arduino.h>

// ── Physical Leg Dimensions (in millimeters) ──────────────────────────────
#define COXA_LENGTH_MM   52.0f   // L1: Hip joint to femur pivot
#define FEMUR_LENGTH_MM  66.0f   // L2: Femur pivot to tibia pivot
#define TIBIA_LENGTH_MM  132.0f  // L3: Tibia pivot to foot tip (ground contact)

// ── Servo Angular Calibration Defaults (in degrees) ──────────────────────
#define COXA_NEUTRAL_DEG   0.0f
#define FEMUR_NEUTRAL_DEG  0.0f
#define TIBIA_NEUTRAL_DEG  0.0f

// ── Joint Angle Hardware Limits (Protection Bounds) ──────────────────────
#define COXA_MIN_DEG   -90.0f
#define COXA_MAX_DEG    90.0f

#define FEMUR_MIN_DEG  -90.0f
#define FEMUR_MAX_DEG   90.0f

#define TIBIA_MIN_DEG  -90.0f
#define TIBIA_MAX_DEG   90.0f

// ── Body Frame Layout Dimensions (in millimeters) ──────────────────────────
#define BODY_LENGTH_MM    173.2f  // 2 * 0.866 * R
#define BODY_WIDTH_CENTER 200.0f  // 2 * R
#define BODY_WIDTH_CORNER 100.0f  // 1 * R
#define LEG_COUNT         6

// ── Leg Mounting Angles (degrees relative to body X-axis) ──────────────────
#define MOUNT_ANGLE_RF  -30.0f   // Leg 0: Right Front
#define MOUNT_ANGLE_RM  -90.0f   // Leg 1: Right Middle
#define MOUNT_ANGLE_RR -150.0f   // Leg 2: Right Rear
#define MOUNT_ANGLE_LR  150.0f   // Leg 3: Left Rear
#define MOUNT_ANGLE_LM   90.0f   // Leg 4: Left Middle
#define MOUNT_ANGLE_LF   30.0f   // Leg 5: Left Front

// ── Default Standing Stance ───────────────────────────────────────────────
#define DEFAULT_FOOT_X   110.0f  // Neutral foot stance outward reach (mm)
#define DEFAULT_FOOT_Y     0.0f  // Neutral foot stance lateral offset (mm)
#define DEFAULT_FOOT_Z  -100.0f  // Neutral foot standing height (mm below hip)