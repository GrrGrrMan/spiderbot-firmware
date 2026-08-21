#include "GaitGenerator.h"
#include <math.h>

GaitGenerator::GaitGenerator() 
    : m_gaitType(GaitType::TRIPOD), 
      m_phaseClock(0.0f) {}

void GaitGenerator::setGaitType(GaitType type) {
    m_gaitType = type;
}

void GaitGenerator::update(float dtSeconds, const VelocityCommand& cmd, LegPosition outputFootTargets[LEG_COUNT]) {
    // Only moving if translational or rotational velocity is actively commanded
    bool isMoving = (fabsf(cmd.vx) > 1.0f || fabsf(cmd.vy) > 1.0f || fabsf(cmd.omega) > 1.0f);

    if (isMoving && cmd.cycleTime > 0.05f) {
        m_phaseClock += (dtSeconds / cmd.cycleTime);
        if (m_phaseClock >= 1.0f) m_phaseClock -= 1.0f;
    } else {
        m_phaseClock = 0.0f; // Instantly lock phase to neutral ground when not walking
    }

    float offsets[LEG_COUNT];
    float swingRatio = 0.5f;

    switch (m_gaitType) {
        case GaitType::RIPPLE:
            swingRatio = 0.333f;
            { const float rip[LEG_COUNT] = { 0.0f, 0.667f, 0.333f, 0.5f, 0.167f, 0.833f }; for(int k=0; k<6; k++) offsets[k] = rip[k]; }
            break;
        case GaitType::WAVE:
            swingRatio = 0.167f;
            { const float wav[LEG_COUNT] = { 0.0f, 0.167f, 0.333f, 0.5f, 0.667f, 0.833f }; for(int k=0; k<6; k++) offsets[k] = wav[k]; }
            break;
        case GaitType::TRIPOD:
        default:
            swingRatio = 0.5f;
            { const float tri[LEG_COUNT] = { 0.0f, 0.5f, 0.5f, 0.0f, 0.0f, 0.5f }; for(int k=0; k<6; k++) offsets[k] = tri[k]; }
            break;
    }

    float stanceRatio = 1.0f - swingRatio;
    const float MOUNT_ANGLES[LEG_COUNT] = { MOUNT_ANGLE_RF, MOUNT_ANGLE_RM, MOUNT_ANGLE_RR, MOUNT_ANGLE_LR, MOUNT_ANGLE_LM, MOUNT_ANGLE_LF };

    for (int i = 0; i < LEG_COUNT; i++) {
        float legRad = cmd.legStance * (M_PI / 180.0f);
        
        float baseFootX = COXA_LENGTH_MM + FEMUR_LENGTH_MM * cosf(legRad);
        float baseFootY = 0.0f;
        float baseFootZ = -FEMUR_LENGTH_MM * sinf(legRad) - TIBIA_LENGTH_MM;

        if (fabsf(cmd.hipStance) > 0.01f) {
            float hipRad = cmd.hipStance * (M_PI / 180.0f);
            float splay = (i == 0 || i == 5) ? hipRad : ((i == 2 || i == 3) ? -hipRad : 0.0f);
            float cosH = cosf(splay), sinH = sinf(splay);
            float rx = baseFootX * cosH - baseFootY * sinH;
            float ry = baseFootX * sinH + baseFootY * cosH;
            baseFootX = rx;
            baseFootY = ry;
        }

        // When stationary (IK Body Pose mode), hold all feet firmly on the ground
        if (!isMoving) {
            outputFootTargets[i].x = baseFootX;
            outputFootTargets[i].y = baseFootY;
            outputFootTargets[i].z = baseFootZ;
            continue;
        }

        // Dynamic walking strides (only computed when actively moving)
        float strideX = cmd.vx * (cmd.cycleTime * stanceRatio);
        float strideY = cmd.vy * (cmd.cycleTime * stanceRatio);

        if (fabsf(cmd.omega) > 0.1f) {
            float yawRad = (cmd.omega * (M_PI / 180.0f)) * (cmd.cycleTime * stanceRatio);
            float mRad = MOUNT_ANGLES[i] * (M_PI / 180.0f);
            
            float hipX = (i == 0 || i == 5) ? BODY_LENGTH_MM/2.0f : ((i == 2 || i == 3) ? -BODY_LENGTH_MM/2.0f : 0.0f);
            float halfW = (i == 1 || i == 4) ? (BODY_WIDTH_CENTER / 2.0f) : (BODY_WIDTH_CORNER / 2.0f);
            float hipY  = (i < 3) ? -halfW : halfW;
            float footWorldX = hipX + cosf(mRad) * baseFootX;
            float footWorldY = hipY + sinf(mRad) * baseFootX;
            
            strideX += -footWorldY * yawRad;
            strideY +=  footWorldX * yawRad;
        }

        float mountRad = MOUNT_ANGLES[i] * (M_PI / 180.0f);
        float cosM = cosf(-mountRad);
        float sinM = sinf(-mountRad);
        
        float localStrideX = strideX * cosM - strideY * sinM;
        float localStrideY = strideX * sinM + strideY * cosM;

        float legPhase = m_phaseClock + offsets[i];
        if (legPhase >= 1.0f) legPhase -= 1.0f;

        if (legPhase < swingRatio) {
            float swingProgress = legPhase / swingRatio;
            outputFootTargets[i].x = baseFootX + (-localStrideX + 2.0f * localStrideX * swingProgress);
            outputFootTargets[i].y = baseFootY + (-localStrideY + 2.0f * localStrideY * swingProgress);
            outputFootTargets[i].z = baseFootZ + (sinf(swingProgress * M_PI) * cmd.stepHeight);
        } else {
            float stanceProgress = (legPhase - swingRatio) / stanceRatio;
            outputFootTargets[i].x = baseFootX + (localStrideX - 2.0f * localStrideX * stanceProgress);
            outputFootTargets[i].y = baseFootY + (localStrideY - 2.0f * localStrideY * stanceProgress);
            outputFootTargets[i].z = baseFootZ;
        }
    }
}