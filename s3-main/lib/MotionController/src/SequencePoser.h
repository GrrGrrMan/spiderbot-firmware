#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include "kinematics_config.h"
#include "TrajectoryEngine.h"
#include "HexapodKinematics.h"

#include "motion_config.h"

struct SequenceKeyframe {
    uint32_t durationMs;
    EasingType easing;
    BodyPose body;
    float alpha[LEG_COUNT];
    float beta[LEG_COUNT];
    float gamma[LEG_COUNT];
    bool overrideJoints[LEG_COUNT];
};

struct PoserOutput {
    BodyPose bodyPose;
    float alpha[LEG_COUNT];
    float beta[LEG_COUNT];
    float gamma[LEG_COUNT];
    bool overrideJoints[LEG_COUNT];
    bool active;
};

class SequencePoser {
public:
    SequencePoser();

    // Loads a dynamic sequence of keyframes from incoming JSON
    bool loadSequence(JsonArrayConst kfArray, uint32_t totalDurationOverrideMs = 0);
    
    void setStartFrame(const BodyPose& pose, const float alpha[LEG_COUNT], const float beta[LEG_COUNT], const float gamma[LEG_COUNT]);
    
    // Evaluates the active sequence at current time tick
    PoserOutput update(uint32_t nowMs);

    void stop();
    bool isActive() const { return m_active; }

private:
    bool m_active;
    uint8_t m_keyframeCount;
    uint8_t m_currentIdx;
    uint32_t m_frameStartTimeMs;

    SequenceKeyframe m_keyframes[MAX_SEQUENCE_KEYFRAMES];
    SequenceKeyframe m_startFrame; // Holds position where current transition started
};