#include "MotionController.h"
#include "servo_config.h"
#include "logger.h"

#define US_PER_DEGREE 11.11f // Angular conversion constant (~1000us span over 90 degrees)
#define MAX_JOINT_DEG_PER_SEC        180.0f // Max servo slew rate, raw/pose mode
#define MAX_POSE_LINEAR_MM_PER_SEC   150.0f // Max slew rate for posX/posY/posZ/legStance/stepHeight
#define MAX_POSE_ANGULAR_DEG_PER_SEC  90.0f // Max slew rate for roll/pitch/yaw/hipStance

// ── UNIFORM HARDWARE POLARITY (Applied equally to all 6 legs) ───────────────
// Set to true or false to match your physical servo horn mounting direction:
const bool INVERT_ALL_COXA  = false; 
const bool INVERT_ALL_FEMUR = true;  // true ensures positive angle pushes body down symmetrically
const bool INVERT_ALL_TIBIA = true;  // true ensures positive angle folds knee inward symmetrically

MotionController::MotionController(ServoManager& servoMgr) 
    : m_servoMgr(servoMgr),
      m_mutex(nullptr) {
    m_mutex = xSemaphoreCreateMutex();
    m_targetPose = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    m_velocityCmd = {0.0f, 0.0f, 0.0f, 25.0f, 1.0f};
    m_currentPose = m_targetPose;
    m_currentVelocity = m_velocityCmd;
    for (int i = 0; i < LEG_COUNT; i++) {
        m_rawTargetAngles[i]  = {0.0f, 0.0f, 0.0f};
        m_rawCurrentAngles[i] = {0.0f, 0.0f, 0.0f};
        m_appliedAngles[i]    = {0.0f, 0.0f, 0.0f};
    }
    m_isRawMode = false; 
}

MotionController::~MotionController() {
    if (m_mutex) {
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
    }
}

void MotionController::begin() {
    if (!m_mutex) {
        m_mutex = xSemaphoreCreateMutex();
    }
    for (int i = 0; i < LEG_COUNT; i++) {
        m_footTargets[i] = { DEFAULT_FOOT_X, DEFAULT_FOOT_Y, DEFAULT_FOOT_Z };
    }
}

void MotionController::setRawServoMode(bool enable) {
    if (m_mutex && xSemaphoreTake(m_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        m_isRawMode = enable;
        xSemaphoreGive(m_mutex);
    } else {
        m_isRawMode = enable;
    }
}

void MotionController::setBodyPose(const BodyPose& pose) {
    if (m_mutex && xSemaphoreTake(m_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        m_targetPose = pose;
        xSemaphoreGive(m_mutex);
    }
}

void MotionController::setVelocity(const VelocityCommand& cmd) {
    if (m_mutex && xSemaphoreTake(m_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        m_velocityCmd = cmd;
        xSemaphoreGive(m_mutex);
    }
}

void MotionController::setGaitType(GaitType type) {
    if (m_mutex && xSemaphoreTake(m_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        m_gaitGen.setGaitType(type);
        xSemaphoreGive(m_mutex);
    }
}

void MotionController::setRawLegAngles(uint8_t leg, float alpha, float beta, float gamma) {
    if (leg >= LEG_COUNT) return;
    if (m_mutex && xSemaphoreTake(m_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        float clampedAlpha = constrain(alpha, COXA_MIN_DEG, COXA_MAX_DEG);
        float clampedBeta  = constrain(beta,  FEMUR_MIN_DEG, FEMUR_MAX_DEG);
        float internalGamma = -gamma;
        float clampedGamma  = constrain(internalGamma, TIBIA_MIN_DEG, TIBIA_MAX_DEG);

        m_rawTargetAngles[leg] = { clampedAlpha, clampedBeta, clampedGamma };
        xSemaphoreGive(m_mutex);
    }
}

void MotionController::getRawLegAngles(uint8_t leg, float& alpha, float& beta, float& gamma) {
    if (leg >= LEG_COUNT) {
        alpha = beta = gamma = 0.0f;
        return;
    }
    if (m_mutex && xSemaphoreTake(m_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        alpha = m_appliedAngles[leg].alpha;
        beta  = m_appliedAngles[leg].beta;
        gamma = m_appliedAngles[leg].gamma;
        xSemaphoreGive(m_mutex);
    } else {
        alpha = m_appliedAngles[leg].alpha;
        beta  = m_appliedAngles[leg].beta;
        gamma = m_appliedAngles[leg].gamma;
    }
}

uint16_t MotionController::degreesToTick(float angleDeg, bool invert, float neutralOffset) {
    if (invert) angleDeg = -angleDeg;
    angleDeg += neutralOffset; 
    
    float pulseUs = 1500.0f + (angleDeg * US_PER_DEGREE);
    pulseUs = constrain(pulseUs, 488.0f, 2393.0f); 
    return (uint16_t)((pulseUs * 4096.0f) / 20000.0f);
}

static inline float slewToward(float current, float target, float maxDelta) {
    float diff = target - current;
    if (diff > maxDelta) diff = maxDelta;
    if (diff < -maxDelta) diff = -maxDelta;
    return current + diff;
}

void MotionController::update(float dtSeconds) {
    if (m_mutex && xSemaphoreTake(m_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;
    }

    float maxLinDelta   = MAX_POSE_LINEAR_MM_PER_SEC * dtSeconds;
    float maxAngDelta   = MAX_POSE_ANGULAR_DEG_PER_SEC * dtSeconds;
    float maxJointDelta = MAX_JOINT_DEG_PER_SEC * dtSeconds;

    if (m_isRawMode) {
        for (uint8_t leg = 0; leg < LEG_COUNT; leg++) {
            m_rawCurrentAngles[leg].alpha = slewToward(m_rawCurrentAngles[leg].alpha, m_rawTargetAngles[leg].alpha, maxJointDelta);
            m_rawCurrentAngles[leg].beta  = slewToward(m_rawCurrentAngles[leg].beta,  m_rawTargetAngles[leg].beta,  maxJointDelta);
            m_rawCurrentAngles[leg].gamma = slewToward(m_rawCurrentAngles[leg].gamma, m_rawTargetAngles[leg].gamma, maxJointDelta);

            m_appliedAngles[leg].alpha = m_rawCurrentAngles[leg].alpha;
            m_appliedAngles[leg].beta  = m_rawCurrentAngles[leg].beta;
            m_appliedAngles[leg].gamma = -m_rawCurrentAngles[leg].gamma;

            // Uniform PWM calculation across ALL 6 legs
            uint16_t coxaWidthTicks  = degreesToTick(m_rawCurrentAngles[leg].alpha, INVERT_ALL_COXA,  COXA_NEUTRAL_DEG);
            uint16_t femurWidthTicks = degreesToTick(m_rawCurrentAngles[leg].beta,  INVERT_ALL_FEMUR, FEMUR_NEUTRAL_DEG);
            uint16_t tibiaWidthTicks = degreesToTick(m_rawCurrentAngles[leg].gamma, INVERT_ALL_TIBIA, TIBIA_NEUTRAL_DEG);

            m_servoMgr.setServoWidthTicks(LEG_COXA_CHANNELS[leg],  coxaWidthTicks);
            m_servoMgr.setServoWidthTicks(LEG_FEMUR_CHANNELS[leg], femurWidthTicks);
            m_servoMgr.setServoWidthTicks(LEG_TIBIA_CHANNELS[leg], tibiaWidthTicks);
        }
        xSemaphoreGive(m_mutex);
        return;
    }

    // --- IK MODE ---
    m_currentPose.posX  = slewToward(m_currentPose.posX,  m_targetPose.posX,  maxLinDelta);
    m_currentPose.posY  = slewToward(m_currentPose.posY,  m_targetPose.posY,  maxLinDelta);
    m_currentPose.posZ  = slewToward(m_currentPose.posZ,  m_targetPose.posZ,  maxLinDelta);
    m_currentPose.roll  = slewToward(m_currentPose.roll,  m_targetPose.roll,  maxAngDelta);
    m_currentPose.pitch = slewToward(m_currentPose.pitch, m_targetPose.pitch, maxAngDelta);
    m_currentPose.yaw   = slewToward(m_currentPose.yaw,   m_targetPose.yaw,   maxAngDelta);

    m_currentVelocity.vx         = slewToward(m_currentVelocity.vx,         m_velocityCmd.vx,         maxLinDelta);
    m_currentVelocity.vy         = slewToward(m_currentVelocity.vy,         m_velocityCmd.vy,         maxLinDelta);
    m_currentVelocity.omega      = slewToward(m_currentVelocity.omega,      m_velocityCmd.omega,      maxAngDelta);
    m_currentVelocity.stepHeight = slewToward(m_currentVelocity.stepHeight, m_velocityCmd.stepHeight, maxLinDelta);
    m_currentVelocity.legStance  = slewToward(m_currentVelocity.legStance,  m_velocityCmd.legStance,  maxLinDelta);
    m_currentVelocity.hipStance  = slewToward(m_currentVelocity.hipStance,  m_velocityCmd.hipStance,  maxAngDelta);
    m_currentVelocity.cycleTime  = m_velocityCmd.cycleTime;

    m_gaitGen.update(dtSeconds, m_currentVelocity, m_footTargets);
    HexapodJoints joints = m_kinematics.computeBodyPose(m_currentPose, m_footTargets);

    for (uint8_t leg = 0; leg < LEG_COUNT; leg++) {
        if (!joints.leg[leg].isValid) continue;

        m_appliedAngles[leg].alpha = joints.leg[leg].coxaDeg;
        m_appliedAngles[leg].beta  = joints.leg[leg].femurDeg;
        m_appliedAngles[leg].gamma = -joints.leg[leg].tibiaDeg;

        // Uniform PWM calculation across ALL 6 legs
        uint16_t coxaWidthTicks  = degreesToTick(joints.leg[leg].coxaDeg, INVERT_ALL_COXA,  COXA_NEUTRAL_DEG);
        uint16_t femurWidthTicks = degreesToTick(joints.leg[leg].femurDeg, INVERT_ALL_FEMUR, FEMUR_NEUTRAL_DEG);
        uint16_t tibiaWidthTicks = degreesToTick(joints.leg[leg].tibiaDeg, INVERT_ALL_TIBIA, TIBIA_NEUTRAL_DEG);

        m_servoMgr.setServoWidthTicks(LEG_COXA_CHANNELS[leg],  coxaWidthTicks);
        m_servoMgr.setServoWidthTicks(LEG_FEMUR_CHANNELS[leg], femurWidthTicks);
        m_servoMgr.setServoWidthTicks(LEG_TIBIA_CHANNELS[leg], tibiaWidthTicks);
    }

    xSemaphoreGive(m_mutex);
}