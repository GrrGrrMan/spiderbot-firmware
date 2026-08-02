#include "HexapodKinematics.h"
#include <math.h>

HexapodKinematics::HexapodKinematics() {
    float halfL = BODY_LENGTH_MM / 2.0f;
    float halfWCorner = BODY_WIDTH_CORNER / 2.0f;
    float halfWCenter = BODY_WIDTH_CENTER / 2.0f;
    
    // Leg Mount Offsets dynamically loaded from kinematics_config.h
    m_mountOffsets[0] = { halfL, -halfWCorner, 0.0f };
    m_mountOffsets[1] = { 0.0f, -halfWCenter, 0.0f };
    m_mountOffsets[2] = { -halfL, -halfWCorner, 0.0f };
    m_mountOffsets[3] = { -halfL, halfWCorner, 0.0f };
    m_mountOffsets[4] = { 0.0f, halfWCenter, 0.0f };
    m_mountOffsets[5] = { halfL, halfWCorner, 0.0f };

    // Configurable Leg Mount Angles dynamically loaded from kinematics_config.h
    m_mountAngles[0] = MOUNT_ANGLE_RF * (M_PI / 180.0f);
    m_mountAngles[1] = MOUNT_ANGLE_RM * (M_PI / 180.0f);
    m_mountAngles[2] = MOUNT_ANGLE_RR * (M_PI / 180.0f);
    m_mountAngles[3] = MOUNT_ANGLE_LR * (M_PI / 180.0f);
    m_mountAngles[4] = MOUNT_ANGLE_LM * (M_PI / 180.0f);
    m_mountAngles[5] = MOUNT_ANGLE_LF * (M_PI / 180.0f);
}


HexapodJoints HexapodKinematics::computeBodyPose(const BodyPose& pose, const LegPosition footTargets[LEG_COUNT]) {
    HexapodJoints output;
    output.allValid = true;

    // 1. Convert Euler angles to Radians
    float rollRad  = pose.roll  * (M_PI / 180.0f);
    float pitchRad = pose.pitch * (M_PI / 180.0f);
    float yawRad   = pose.yaw   * (M_PI / 180.0f);

    // 2. Precompute Trigonometry for 3D Rotation Matrix
    float cosR = cosf(rollRad),  sinR = sinf(rollRad);
    float cosP = cosf(pitchRad), sinP = sinf(pitchRad);
    float cosY = cosf(yawRad),   sinY = sinf(yawRad);

    for (int i = 0; i < LEG_COUNT; i++) {
        // Total foot position relative to body center
        float totalFootX = m_mountOffsets[i].x + footTargets[i].x;
        float totalFootY = m_mountOffsets[i].y + footTargets[i].y;
        float totalFootZ = m_mountOffsets[i].z + footTargets[i].z;

        // Apply 3D Rotation Matrix (Roll, Pitch, Yaw)
        float rotX = cosY * cosP * totalFootX + 
                     (cosY * sinP * sinR - sinY * cosR) * totalFootY + 
                     (cosY * sinP * cosR + sinY * sinR) * totalFootZ;

        float rotY = sinY * cosP * totalFootX + 
                     (sinY * sinP * sinR + cosY * cosR) * totalFootY + 
                     (sinY * sinP * cosR - cosY * sinR) * totalFootZ;

        float rotZ = -sinP * totalFootX + 
                     cosP * sinR * totalFootY + 
                     cosP * cosR * totalFootZ;

        // Apply body position translation shift
        float bodyShiftX = rotX - pose.posX - m_mountOffsets[i].x;
        float bodyShiftY = rotY - pose.posY - m_mountOffsets[i].y;
        float bodyShiftZ = rotZ - pose.posZ - m_mountOffsets[i].z;

        // Transform into individual leg's local coordinate frame using mounting angle
        float cosM = cosf(-m_mountAngles[i]);
        float sinM = sinf(-m_mountAngles[i]);

        float legLocalX = bodyShiftX * cosM - bodyShiftY * sinM;
        float legLocalY = bodyShiftX * sinM + bodyShiftY * cosM;
        float legLocalZ = bodyShiftZ;

        // Solve 3-DOF IK for the current leg
        output.leg[i] = m_legIK.solveIK(legLocalX, legLocalY, legLocalZ);

        if (!output.leg[i].isValid) {
            output.allValid = false;
        }
    }

    return output;
}