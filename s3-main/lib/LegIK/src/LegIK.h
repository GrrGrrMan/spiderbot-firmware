#pragma once

#include <Arduino.h>
#include "kinematics_config.h"

// Output struct containing computed joint angles in degrees
struct LegAngles {
    float coxaDeg;
    float femurDeg;
    float tibiaDeg;
    bool isValid; // Returns false if targeted (x, y, z) is physically out of reach
};

class LegIK {
public:
    LegIK();

    // Solves 3-DOF Inverse Kinematics for a given target foot position (x, y, z) relative to leg root
    LegAngles solveIK(float targetX, float targetY, float targetZ);

private:
    float m_l1; // Coxa length
    float m_l2; // Femur length
    float m_l3; // Tibia length
};