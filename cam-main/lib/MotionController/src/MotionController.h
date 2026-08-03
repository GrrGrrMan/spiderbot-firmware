#pragma once

#include <Arduino.h>
#include "ServoManager.h"
#include "HexapodKinematics.h"
#include "GaitGenerator.h"

class MotionController {
public:
    MotionController(ServoManager& servoMgr);

    void begin();
    
    // Executed deterministically at 100Hz inside TaskControl on Core 1
    void update(float dtSeconds);

    void setBodyPose(const BodyPose& pose);
    void setVelocity(const VelocityCommand& cmd);
    void setGaitType(GaitType type);
    void setRawServoMode(bool enable);


private:
    bool m_isRawMode;
    ServoManager& m_servoMgr;
    HexapodKinematics m_kinematics;
    GaitGenerator m_gaitGen;

    BodyPose m_targetPose;
    VelocityCommand m_velocityCmd;
    LegPosition m_footTargets[LEG_COUNT];

    uint16_t degreesToTick(float angleDeg, bool invert);
    
};