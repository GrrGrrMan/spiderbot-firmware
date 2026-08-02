#pragma once

#include <Arduino.h>
#include "LegIK.h"
#include "kinematics_config.h"

// 6-DOF Body Pose Target Structure
struct BodyPose {
    float posX;   // Shift X (mm)
    float posY;   // Shift Y (mm)
    float posZ;   // Height offset (mm)
    float roll;   // Body roll angle (degrees)
    float pitch;  // Body pitch angle (degrees)
    float yaw;    // Body yaw angle (degrees)
};

// Target 3D coordinate for an individual foot
struct LegPosition {
    float x;
    float y;
    float z;
};

// Container for all 18 solved joint angles across 6 legs
struct HexapodJoints {
    LegAngles leg[LEG_COUNT];
    bool allValid;
};

class HexapodKinematics {
public:
    HexapodKinematics();

    // Calculates solved joint angles for all 6 legs under body pose shift and foot placement targets
    HexapodJoints computeBodyPose(const BodyPose& pose, const LegPosition footTargets[LEG_COUNT]);

private:
    LegIK m_legIK;
    LegPosition m_mountOffsets[LEG_COUNT]; // Mounting X, Y, Z offsets relative to body center
    float m_mountAngles[LEG_COUNT];        // Mounting rotation angles relative to body X-axis
};