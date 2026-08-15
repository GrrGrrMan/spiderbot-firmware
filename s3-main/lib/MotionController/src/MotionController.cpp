#include "MotionController.h"
#include "servo_config.h"
#include "logger.h"

#define US_PER_DEGREE 11.11f // Angular conversion constant (~1000us span over 90 degrees)
#define MAX_JOINT_DEG_PER_SEC        180.0f // Max servo slew rate, raw/pose mode
#define MAX_POSE_LINEAR_MM_PER_SEC   150.0f // Max slew rate for posX/posY/posZ/legStance/stepHeight
#define MAX_POSE_ANGULAR_DEG_PER_SEC  90.0f // Max slew rate for roll/pitch/yaw/hipStance

MotionController::MotionController(ServoManager& servoMgr) 
    : m_servoMgr(servoMgr) {
    m_targetPose = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    m_velocityCmd = {0.0f, 0.0f, 0.0f, 25.0f, 1.0f}; // Default: 25mm lift, 1.0s stride cycle
    m_currentPose = m_targetPose;
    m_currentVelocity = m_velocityCmd;
    for (int i = 0; i < LEG_COUNT; i++) {
        m_rawTargetAngles[i]  = {0.0f, 0.0f, 0.0f};
        m_rawCurrentAngles[i] = {0.0f, 0.0f, 0.0f};
    }
    m_isRawMode = false; 
}

void MotionController::setRawServoMode(bool enable) {
    m_isRawMode = enable;
}

void MotionController::begin() {
    for (int i = 0; i < LEG_COUNT; i++) {
        m_footTargets[i] = { DEFAULT_FOOT_X, DEFAULT_FOOT_Y, DEFAULT_FOOT_Z };
    }
}

void MotionController::setBodyPose(const BodyPose& pose) {
    m_targetPose = pose;
}

void MotionController::setVelocity(const VelocityCommand& cmd) {
    m_velocityCmd = cmd;
}

void MotionController::setGaitType(GaitType type) {
    m_gaitGen.setGaitType(type);
}

uint16_t MotionController::degreesToTick(float angleDeg, bool invert, float neutralOffset) {
    if (invert) angleDeg = -angleDeg;
    angleDeg += neutralOffset; // Inject physical hardware calibration trim
    
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
    float maxLinDelta = MAX_POSE_LINEAR_MM_PER_SEC * dtSeconds;
    float maxAngDelta = MAX_POSE_ANGULAR_DEG_PER_SEC * dtSeconds;
    float maxJointDelta = MAX_JOINT_DEG_PER_SEC * dtSeconds;

    if (m_isRawMode) {
        // Slew each leg's raw angles toward their MQTT-commanded targets,
        // decoupled from however fast pose messages actually arrive.
        for (uint8_t leg = 0; leg < LEG_COUNT; leg++) {
            m_rawCurrentAngles[leg].alpha = slewToward(m_rawCurrentAngles[leg].alpha, m_rawTargetAngles[leg].alpha, maxJointDelta);
            m_rawCurrentAngles[leg].beta  = slewToward(m_rawCurrentAngles[leg].beta,  m_rawTargetAngles[leg].beta,  maxJointDelta);
            m_rawCurrentAngles[leg].gamma = slewToward(m_rawCurrentAngles[leg].gamma, m_rawTargetAngles[leg].gamma, maxJointDelta);

            bool invertLeg = (leg >= 3);
            uint16_t coxaWidthTicks  = degreesToTick(m_rawCurrentAngles[leg].alpha, invertLeg, COXA_NEUTRAL_DEG);
            uint16_t femurWidthTicks = degreesToTick(m_rawCurrentAngles[leg].beta,  invertLeg, FEMUR_NEUTRAL_DEG);
            uint16_t tibiaWidthTicks = degreesToTick(m_rawCurrentAngles[leg].gamma, invertLeg, TIBIA_NEUTRAL_DEG);

            m_servoMgr.setServoWidthTicks(LEG_COXA_CHANNELS[leg],  coxaWidthTicks);
            m_servoMgr.setServoWidthTicks(LEG_FEMUR_CHANNELS[leg], femurWidthTicks);
            m_servoMgr.setServoWidthTicks(LEG_TIBIA_CHANNELS[leg], tibiaWidthTicks);
        }
        return;
    }

    // 0. Slew current pose/velocity toward their commanded targets
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
    m_currentVelocity.cycleTime  = m_velocityCmd.cycleTime; // timing constant, safe to apply immediately

    // 1. Advance Gait Generator foot trajectories if velocity vector is active
    if (fabsf(m_currentVelocity.vx) > 0.1f || fabsf(m_currentVelocity.vy) > 0.1f || fabsf(m_currentVelocity.omega) > 0.1f) {
        m_gaitGen.update(dtSeconds, m_currentVelocity, m_footTargets);
    }

    // 2. Calculate 6-leg body pose + leg IK angles
    HexapodJoints joints = m_kinematics.computeBodyPose(m_currentPose, m_footTargets);

    static unsigned long lastIkLog = 0;
    bool printedError = false;

    // 3. Map solved angles to physical PCA9685 channels across dual boards
    for (uint8_t leg = 0; leg < LEG_COUNT; leg++) {
        if (!joints.leg[leg].isValid) {
            if (millis() - lastIkLog > 1000 && !printedError) {
                LOG_ERR("IK Failsafe tripped on Leg %d!", leg);
                printedError = true;
                lastIkLog = millis();
            }
            continue;
        }

        uint8_t coxaCh  = LEG_COXA_CHANNELS[leg];
        uint8_t femurCh = LEG_FEMUR_CHANNELS[leg];
        uint8_t tibiaCh = LEG_TIBIA_CHANNELS[leg];
        bool invertLeg = (leg >= 3);

        uint16_t coxaWidthTicks  = degreesToTick(joints.leg[leg].coxaDeg, invertLeg, COXA_NEUTRAL_DEG);
        uint16_t femurWidthTicks = degreesToTick(joints.leg[leg].femurDeg, invertLeg, FEMUR_NEUTRAL_DEG);
        uint16_t tibiaWidthTicks = degreesToTick(joints.leg[leg].tibiaDeg, invertLeg, TIBIA_NEUTRAL_DEG);

        m_servoMgr.setServoWidthTicks(coxaCh,  coxaWidthTicks);
        m_servoMgr.setServoWidthTicks(femurCh, femurWidthTicks);
        m_servoMgr.setServoWidthTicks(tibiaCh, tibiaWidthTicks);
    }
}

void MotionController::setRawLegAngles(uint8_t leg, float alpha, float beta, float gamma) {
    // Note: The Web UI gamma (tibia) aligns negatively to the math model.
    // Only stores the target now — update() slews toward it and writes servos.
    m_rawTargetAngles[leg] = { alpha, beta, -gamma };
}