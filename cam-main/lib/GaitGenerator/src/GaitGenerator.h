#pragma once

#include <Arduino.h>
#include "HexapodKinematics.h"
#include "kinematics_config.h"

enum class GaitType {
    TRIPOD,
    RIPPLE,
    WAVE
};

struct VelocityCommand {
    float vx;          // Forward/Backward velocity (mm/s)
    float vy;          // Lateral side-step velocity (mm/s)
    float omega;       // Yaw turning rate (degrees/s)
    float stepHeight;  // Max foot lift height during swing phase (mm)
    float cycleTime;   // Full gait cycle duration (seconds)
    float legStance;   // Outward stance distance offset (mm, default 0)
    float hipStance;   // Hip angle splay offset (degrees, default 0)
};

class GaitGenerator {
public:
    GaitGenerator();

    // Advances gait phase clock by dtSeconds and outputs 3D foot targets for all 6 legs
    void update(float dtSeconds, const VelocityCommand& cmd, LegPosition outputFootTargets[LEG_COUNT]);

    void setGaitType(GaitType type);

private:
    GaitType m_gaitType;
    float m_phaseClock; // Normalized gait cycle progress [0.0 to 1.0)
};