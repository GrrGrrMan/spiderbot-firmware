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
        // Convert the incoming Local Foot Target into the Global Body Frame!
        float cosM_fwd = cosf(m_mountAngles[i]);
        float sinM_fwd = sinf(m_mountAngles[i]);
        
        float hipFootBodyX = footTargets[i].x * cosM_fwd - footTargets[i].y * sinM_fwd;
        float hipFootBodyY = footTargets[i].x * sinM_fwd + footTargets[i].y * cosM_fwd;

        // Total foot position relative to body center in the Body Frame
        float totalFootX = m_mountOffsets[i].x + hipFootBodyX;
        float totalFootY = m_mountOffsets[i].y + hipFootBodyY;
        float totalFootZ = m_mountOffsets[i].z + footTargets[i].z;

        // Inverse rotation matrix (R^T)
        float rotX = cosY * cosP * (totalFootX - pose.posX) + 
                    sinY * cosP * (totalFootY - pose.posY) - 
                    sinP        * (totalFootZ - pose.posZ);

        float rotY = (cosY * sinP * sinR - sinY * cosR) * (totalFootX - pose.posX) + 
                    (sinY * sinP * sinR + cosY * cosR) * (totalFootY - pose.posY) + 
                    cosP * sinR                       * (totalFootZ - pose.posZ);

        float rotZ = (cosY * sinP * cosR + sinY * sinR) * (totalFootX - pose.posX) + 
                    (sinY * sinP * cosR - cosY * sinR) * (totalFootY - pose.posY) + 
                    cosP * cosR                       * (totalFootZ - pose.posZ);

        float bodyShiftX = rotX - m_mountOffsets[i].x;
        float bodyShiftY = rotY - m_mountOffsets[i].y;
        float bodyShiftZ = rotZ - m_mountOffsets[i].z;

        // Transform back into individual leg's local coordinate frame using inverse mounting angle
        float cosM_inv = cosf(-m_mountAngles[i]);
        float sinM_inv = sinf(-m_mountAngles[i]);

        float legLocalX = bodyShiftX * cosM_inv - bodyShiftY * sinM_inv;
        float legLocalY = bodyShiftX * sinM_inv + bodyShiftY * cosM_inv;
        float legLocalZ = bodyShiftZ;

        // Solve 3-DOF IK for the current leg
        output.leg[i] = m_legIK.solveIK(legLocalX, legLocalY, legLocalZ);

        if (!output.leg[i].isValid) {
            output.allValid = false;
        }
    }

    return output;
}