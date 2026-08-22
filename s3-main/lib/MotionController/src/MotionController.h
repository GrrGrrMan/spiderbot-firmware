#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "ServoManager.h"
#include "HexapodKinematics.h"
#include "GaitGenerator.h"
#include "SequencePoser.h"

class MotionController {
public:
    MotionController(ServoManager& servoMgr);
    ~MotionController();

    void begin();
    void update(float dtSeconds);

    // Dynamic sequence loader (receives dynamic keyframe arrays over MQTT)
    void playSequence(JsonArrayConst keyframes, uint32_t durationOverrideMs = 0);
    void stopSequence();
    bool isSequenceActive() const;

    // Standard motion / pose controls
    void setBodyPose(const BodyPose& pose);
    void setVelocity(const VelocityCommand& cmd);
    void setGaitType(GaitType type);
    void setRawServoMode(bool enable);
    void setRawLegAngles(uint8_t leg, float alpha, float beta, float gamma);
    void getRawLegAngles(uint8_t leg, float& alpha, float& beta, float& gamma);

private:
    bool m_isRawMode;
    ServoManager& m_servoMgr;
    HexapodKinematics m_kinematics;
    GaitGenerator m_gaitGen;
    SequencePoser m_sequencePoser;
    SemaphoreHandle_t m_mutex;

    BodyPose m_targetPose;
    BodyPose m_currentPose;           
    VelocityCommand m_velocityCmd;
    VelocityCommand m_currentVelocity; 
    LegPosition m_footTargets[LEG_COUNT];

    struct RawLegAngles { float alpha, beta, gamma; };
    RawLegAngles m_rawTargetAngles[LEG_COUNT];  
    RawLegAngles m_rawCurrentAngles[LEG_COUNT]; 
    RawLegAngles m_appliedAngles[LEG_COUNT];    

    float m_softStartElapsed;                   

    uint16_t degreesToTick(float angleDeg, bool invert, float neutralOffset);
};