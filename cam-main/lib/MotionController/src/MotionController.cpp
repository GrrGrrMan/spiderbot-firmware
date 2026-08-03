#include "MotionController.h"
#include "servo_config.h"
#include "logger.h"

#define US_PER_DEGREE 11.11f // Angular conversion constant (~1000us span over 90 degrees)

MotionController::MotionController(ServoManager& servoMgr) 
    : m_servoMgr(servoMgr) {
    m_targetPose = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    m_velocityCmd = {0.0f, 0.0f, 0.0f, 25.0f, 1.0f}; // Default: 25mm lift, 1.0s stride cycle
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

uint16_t MotionController::degreesToTick(float angleDeg, bool invert) {
    if (invert) angleDeg = -angleDeg;
    float pulseUs = 1500.0f + (angleDeg * US_PER_DEGREE);
    pulseUs = constrain(pulseUs, 500.0f, 2500.0f); // Hardware pulse limits
    return (uint16_t)((pulseUs * 4096.0f) / 20000.0f);
}

void MotionController::update(float dtSeconds) {
    // Yield hardware control if we are receiving direct manual servo commands
    if (m_isRawMode) return; 

    // 1. Advance Gait Generator foot trajectories if velocity vector is active
    if (fabsf(m_velocityCmd.vx) > 0.1f || fabsf(m_velocityCmd.vy) > 0.1f || fabsf(m_velocityCmd.omega) > 0.1f) {
        m_gaitGen.update(dtSeconds, m_velocityCmd, m_footTargets);
    }

    // 2. Calculate 6-leg body pose + leg IK angles
    HexapodJoints joints = m_kinematics.computeBodyPose(m_targetPose, m_footTargets);

    // REMOVED: if (!joints.allValid) return; (This was causing the silent freeze!)

    // Static variables to throttle our error logs so we don't spam the console at 100Hz
    static unsigned long lastIkLog = 0;
    bool printedError = false;

    // 3. Map solved angles to physical PCA9685 channels across dual boards
    for (uint8_t leg = 0; leg < LEG_COUNT; leg++) {
        
        // If this specific leg failed the IK math, log it and skip ONLY this leg
        if (!joints.leg[leg].isValid) {
            if (millis() - lastIkLog > 1000 && !printedError) {
                LOG_ERR("IK Failsafe tripped on Leg %d! Coordinate is physically out of reach.", leg);
                printedError = true;
                lastIkLog = millis();
            }
            continue; 
        }

        uint8_t coxaCh  = LEG_COXA_CHANNELS[leg];
        uint8_t femurCh = LEG_FEMUR_CHANNELS[leg];
        uint8_t tibiaCh = LEG_TIBIA_CHANNELS[leg];

        bool invertLeg = (leg >= 3); // Invert left side legs for mirrored symmetry

        // Retrieve raw joint duration ticks (neutral is ~307 ticks / 1500us)
        uint16_t coxaWidthTicks  = degreesToTick(joints.leg[leg].coxaDeg, invertLeg);
        uint16_t femurWidthTicks = degreesToTick(joints.leg[leg].femurDeg, invertLeg);
        uint16_t tibiaWidthTicks = degreesToTick(joints.leg[leg].tibiaDeg, invertLeg);

        // Clamp staggering start points inside the 12-bit clock limit (0 - 4095)
        uint16_t coxaOn   = (coxaCh  * STAGGER_OFFSET) % 4096;
        uint16_t femurOn  = (femurCh * STAGGER_OFFSET) % 4096;
        uint16_t tibiaOn  = (tibiaCh * STAGGER_OFFSET) % 4096;

        // Calculate absolute turn-off positions wrapping around the 4096-tick window
        uint16_t coxaOff  = (coxaOn  + coxaWidthTicks) % 4096;
        uint16_t femurOff = (femurOn + femurWidthTicks) % 4096;
        uint16_t tibiaOff = (tibiaOn + tibiaWidthTicks) % 4096;

        m_servoMgr.setPWM(coxaCh,  coxaOn,  coxaOff);
        m_servoMgr.setPWM(femurCh, femurOn, femurOff);
        m_servoMgr.setPWM(tibiaCh, tibiaOn, tibiaOff);
    }
}