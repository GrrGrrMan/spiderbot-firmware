#pragma once

#include <Arduino.h>

// ── Motion Watchdog & Safety Timeouts ───────────────────────────────────────
#define MOTION_WATCHDOG_TIMEOUT_MS   3000   // Stage 1: Auto-brake gait velocity to 0 after 3.0s
#define INACTIVITY_SLEEP_TIMEOUT_MS 15000   // Stage 2: Pull OE HIGH (LIMP sleep) after 15s of total inactivity
#define MAX_CONTINUOUS_MOTION_MS        0   // 0 = Disabled (safety against packet drop is handled by MOTION_WATCHDOG_TIMEOUT_MS)

// ── Dynamic Keyframe Sequence Limits ────────────────────────────────────────
#define MAX_SEQUENCE_KEYFRAMES         32   // Expanded capacity for rich choreography (dances/transitions)
#define DEFAULT_KEYFRAME_DUR_MS       400   // Default duration if keyframe duration is unspecified
#define MIN_KEYFRAME_SEGMENT_MS        40   // Hard minimum time per keyframe segment to prevent servo stall

// ── Slew Rate & Speed Limits ────────────────────────────────────────────────
#define MAX_JOINT_DEG_PER_SEC       240.0f  // Max angular velocity per servo joint
#define SOFT_START_JOINT_DEG_PER_SEC 50.0f  // Initial speed cap during soft-start
#define SOFT_START_DURATION_SEC       1.5f  // Duration of startup glide-in
#define MAX_POSE_LINEAR_MM_PER_SEC  220.0f  // Max 6-DoF body translation slew rate
#define MAX_POSE_ANGULAR_DEG_PER_SEC 180.0f // Max 6-DoF body rotation slew rate (crisp tilt/yaw gestures)

// ── Servo Conversion Factor ─────────────────────────────────────────────────
#define US_PER_DEGREE               11.11f  // Microseconds per degree pulse width