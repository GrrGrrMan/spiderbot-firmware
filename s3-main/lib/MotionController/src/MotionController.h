#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "ServoManager.h"
#include "HexapodKinematics.h"
#include "GaitGenerator.h"

class MotionController {
public:
    MotionController(ServoManager& servoMgr);
    ~MotionController();

    void begin();
    
    // Executed deterministically at 100Hz inside TaskControl on Core 1
    void update(float dtSeconds);

    // Thread-safe command setters (invoked from Core 0 / TaskNetwork)
    void setBodyPose(const BodyPose& pose);
    void setVelocity(const VelocityCommand& cmd);
    void setGaitType(GaitType type);
    void setRawServoMode(bool enable);
    void setRawLegAngles(uint8_t leg, float alpha, float beta, float gamma);

    // Thread-safe readback for MQTT telemetry on Core 0 (Ghost Silhouette)
    void getRawLegAngles(uint8_t leg, float& alpha, float& beta, float& gamma);

private:
    bool m_isRawMode;
    ServoManager& m_servoMgr;
    HexapodKinematics m_kinematics;
    GaitGenerator m_gaitGen;
    SemaphoreHandle_t m_mutex;

    BodyPose m_targetPose;
    BodyPose m_currentPose;           // Slewed pose actually applied each tick
    VelocityCommand m_velocityCmd;
    VelocityCommand m_currentVelocity; // Slewed velocity actually applied each tick
    LegPosition m_footTargets[LEG_COUNT];

    struct RawLegAngles { float alpha, beta, gamma; };
    RawLegAngles m_rawTargetAngles[LEG_COUNT];  // Latest MQTT-commanded raw pose per leg
    RawLegAngles m_rawCurrentAngles[LEG_COUNT]; // Slewed raw angles actually written each tick
    RawLegAngles m_appliedAngles[LEG_COUNT];    // Actual output angles for both IK and Raw modes

    float m_softStartElapsed;                   // Seconds elapsed since startup for smooth glide-in

    uint16_t degreesToTick(float angleDeg, bool invert, float neutralOffset);
};