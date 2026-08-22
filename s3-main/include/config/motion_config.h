#pragma once

#include <Arduino.h>

// ── Motion Watchdog & Safety Timeouts ───────────────────────────────────────
#define MOTION_WATCHDOG_TIMEOUT_MS   3000   // Auto-brake walk velocity to 0 after 3.0s of no motion updates
#define INACTIVITY_SLEEP_TIMEOUT_MS  6000   // Power down servos (LIMP / Sleep) after 6s of complete idle
#define MAX_CONTINUOUS_MOTION_MS    15000   // Hard cutoff for any single continuous movement block

// ── Dynamic Keyframe Sequence Limits ────────────────────────────────────────
#define MAX_SEQUENCE_KEYFRAMES         16   // Maximum keyframes allowed in one dynamic sequence
#define DEFAULT_KEYFRAME_DUR_MS       400   // Default duration if keyframe duration is unspecified
#define MIN_KEYFRAME_SEGMENT_MS        40   // Hard minimum time per keyframe segment to prevent servo stall

// ── Slew Rate & Speed Limits ────────────────────────────────────────────────
#define MAX_JOINT_DEG_PER_SEC       220.0f  // Max angular velocity per servo joint
#define SOFT_START_JOINT_DEG_PER_SEC 40.0f  // Initial speed cap during soft-start
#define SOFT_START_DURATION_SEC       2.0f  // Duration of startup glide-in
#define MAX_POSE_LINEAR_MM_PER_SEC  160.0f  // Max 6-DoF body translation slew rate
#define MAX_POSE_ANGULAR_DEG_PER_SEC 100.0f // Max 6-DoF body rotation slew rate

// ── Servo Conversion Factor ─────────────────────────────────────────────────
#define US_PER_DEGREE               11.11f  // Microseconds per degree pulse width