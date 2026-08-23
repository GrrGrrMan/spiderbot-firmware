#include "SequencePoser.h"
#include "logger.h"

static EasingType parseEasingStr(const char* str) {
    if (!str) return EasingType::EASE_IN_OUT_CUBIC;
    if (strcmp(str, "linear") == 0) return EasingType::LINEAR;
    if (strcmp(str, "easeInOutQuad") == 0 || strcmp(str, "quad") == 0) return EasingType::EASE_IN_OUT_QUAD;
    if (strcmp(str, "easeInOutSine") == 0 || strcmp(str, "sine") == 0) return EasingType::EASE_IN_OUT_SINE;
    if (strcmp(str, "minJerk") == 0 || strcmp(str, "quintic") == 0) return EasingType::MINIMUM_JERK;
    return EasingType::EASE_IN_OUT_CUBIC;
}

SequencePoser::SequencePoser()
    : m_active(false),
      m_keyframeCount(0),
      m_currentIdx(0),
      m_frameStartTimeMs(0) {}

bool SequencePoser::loadSequence(JsonArrayConst kfArray, uint32_t totalDurationOverrideMs) {
    if (kfArray.isNull() || kfArray.size() == 0) return false;

    m_keyframeCount = min((size_t)kfArray.size(), (size_t)MAX_SEQUENCE_KEYFRAMES);
    const char* legKeys[6] = { "rf", "rm", "rr", "lr", "lm", "lf" };

    uint32_t originalTotalMs = 0;
    for (uint8_t i = 0; i < m_keyframeCount; i++) {
        JsonObjectConst kfObj = kfArray[i];
        originalTotalMs += (kfObj["duration_ms"] | (kfObj["dur"] | 400));
    }

    float scale = (totalDurationOverrideMs > 0 && originalTotalMs > 0)
        ? ((float)totalDurationOverrideMs / (float)originalTotalMs)
        : 1.0f;

    for (uint8_t i = 0; i < m_keyframeCount; i++) {
        JsonObjectConst kfObj = kfArray[i];
        SequenceKeyframe& kf = m_keyframes[i];

        uint32_t baseDur = kfObj["duration_ms"] | (kfObj["dur"] | 400);
        kf.durationMs = max((uint32_t)40, (uint32_t)(baseDur * scale));
        kf.easing = parseEasingStr(kfObj["easing"] | kfObj["ease"]);

        kf.body.posX  = kfObj["tx"] | 0.0f;
        kf.body.posY  = kfObj["ty"] | 0.0f;
        kf.body.posZ  = kfObj["tz"] | 0.0f;
        kf.body.roll  = -(kfObj["rx"] | (kfObj["roll"]  | 0.0f));
        kf.body.pitch = -(kfObj["ry"] | (kfObj["pitch"] | 0.0f));
        kf.body.yaw   = -(kfObj["rz"] | (kfObj["yaw"]   | 0.0f));

        for (int l = 0; l < LEG_COUNT; l++) {
            kf.overrideJoints[l] = false;
            kf.alpha[l] = 0.0f;
            kf.beta[l]  = 0.0f;
            kf.gamma[l] = 0.0f;
        }

        if (kfObj["joints"].is<JsonObjectConst>()) {
            JsonObjectConst jObj = kfObj["joints"].as<JsonObjectConst>();
            for (uint8_t l = 0; l < 6; l++) {
                if (jObj[legKeys[l]].is<JsonObjectConst>()) {
                    kf.overrideJoints[l] = true;
                    kf.alpha[l] = jObj[legKeys[l]]["alpha"] | 0.0f;
                    kf.beta[l]  = jObj[legKeys[l]]["beta"]  | 0.0f;
                    kf.gamma[l] = jObj[legKeys[l]]["gamma"] | 0.0f;
                }
            }
        }
    }

    m_currentIdx = 0;
    m_frameStartTimeMs = millis();
    m_active = true;

    // Initialize start frame at neutral (can be overridden by setStartFrame)
    m_startFrame.body = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    for (int l = 0; l < LEG_COUNT; l++) {
        m_startFrame.overrideJoints[l] = false;
        m_startFrame.alpha[l] = 0.0f;
        m_startFrame.beta[l]  = 0.0f;
        m_startFrame.gamma[l] = 0.0f;
    }

    LOG_MOT("SequencePoser: Loaded %u dynamic keyframes", m_keyframeCount);
    return true;
}

void SequencePoser::setStartFrame(const BodyPose& pose, const float alpha[LEG_COUNT], const float beta[LEG_COUNT], const float gamma[LEG_COUNT]) {
    m_startFrame.body = pose;
    for (int l = 0; l < LEG_COUNT; l++) {
        // Leave overrideJoints alone (defaults to false), but store current angles
        // so if the target keyframe uses joint overrides, it interpolates smoothly from the current physical stance.
        m_startFrame.alpha[l] = alpha[l];
        m_startFrame.beta[l]  = beta[l];
        m_startFrame.gamma[l] = gamma[l];
    }
}

PoserOutput SequencePoser::update(uint32_t nowMs) {
    PoserOutput out;
    out.active = m_active;

    if (!m_active || m_keyframeCount == 0) {
        out.active = false;
        return out;
    }

    SequenceKeyframe& target = m_keyframes[m_currentIdx];
    uint32_t elapsed = nowMs - m_frameStartTimeMs;
    float tau = (target.durationMs > 0) ? ((float)elapsed / (float)target.durationMs) : 1.0f;
    tau = constrain(tau, 0.0f, 1.0f);

    // Interpolate 6-DoF Body Pose
    out.bodyPose.posX  = TrajectoryEngine::interpolate(m_startFrame.body.posX,  target.body.posX,  tau, target.easing);
    out.bodyPose.posY  = TrajectoryEngine::interpolate(m_startFrame.body.posY,  target.body.posY,  tau, target.easing);
    out.bodyPose.posZ  = TrajectoryEngine::interpolate(m_startFrame.body.posZ,  target.body.posZ,  tau, target.easing);
    out.bodyPose.roll  = TrajectoryEngine::interpolate(m_startFrame.body.roll,  target.body.roll,  tau, target.easing);
    out.bodyPose.pitch = TrajectoryEngine::interpolate(m_startFrame.body.pitch, target.body.pitch, tau, target.easing);
    out.bodyPose.yaw   = TrajectoryEngine::interpolate(m_startFrame.body.yaw,   target.body.yaw,   tau, target.easing);

    // Interpolate Joint Overrides
    for (int l = 0; l < LEG_COUNT; l++) {
        out.overrideJoints[l] = target.overrideJoints[l] || m_startFrame.overrideJoints[l];
        if (out.overrideJoints[l]) {
            out.alpha[l] = TrajectoryEngine::interpolate(m_startFrame.alpha[l], target.alpha[l], tau, target.easing);
            out.beta[l]  = TrajectoryEngine::interpolate(m_startFrame.beta[l],  target.beta[l],  tau, target.easing);
            out.gamma[l] = TrajectoryEngine::interpolate(m_startFrame.gamma[l], target.gamma[l], tau, target.easing);
        }
    }

    // Advance to next keyframe in sequence when current duration expires
    if (tau >= 1.0f) {
        m_startFrame = target;
        m_currentIdx++;
        m_frameStartTimeMs = nowMs;

        if (m_currentIdx >= m_keyframeCount) {
            m_active = false;
            out.active = false;
            LOG_MOT("SequencePoser: Sequence playback finished.");
        }
    }

    return out;
}

void SequencePoser::stop() {
    m_active = false;
    m_keyframeCount = 0;
}