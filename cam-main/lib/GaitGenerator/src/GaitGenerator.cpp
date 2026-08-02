#include "GaitGenerator.h"
#include <math.h>

GaitGenerator::GaitGenerator() 
    : m_gaitType(GaitType::TRIPOD), 
      m_phaseClock(0.0f) {}

void GaitGenerator::setGaitType(GaitType type) {
    m_gaitType = type;
}

void GaitGenerator::update(float dtSeconds, const VelocityCommand& cmd, LegPosition outputFootTargets[LEG_COUNT]) {
    if (cmd.cycleTime > 0.05f) {
        m_phaseClock += (dtSeconds / cmd.cycleTime);
        if (m_phaseClock >= 1.0f) m_phaseClock -= 1.0f;
    }

    float offsets[LEG_COUNT];
    float swingRatio = 0.5f; // Fraction of stride cycle spent in the air

    // Select phase offsets and swing/stance duty ratios based on GaitType
    switch (m_gaitType) {
        case GaitType::RIPPLE: {
            // Ripple: 2 legs in air at a time (33.3% swing, 66.7% stance)
            swingRatio = 0.333f;
            const float rippleOffsets[LEG_COUNT] = { 0.0f, 0.667f, 0.333f, 0.5f, 0.167f, 0.833f };
            for (int k = 0; k < LEG_COUNT; k++) offsets[k] = rippleOffsets[k];
            break;
        }
        case GaitType::WAVE: {
            // Wave: 1 leg in air at a time (16.7% swing, 83.3% stance)
            swingRatio = 0.167f;
            const float waveOffsets[LEG_COUNT] = { 0.0f, 0.167f, 0.333f, 0.5f, 0.667f, 0.833f };
            for (int k = 0; k < LEG_COUNT; k++) offsets[k] = waveOffsets[k];
            break;
        }
        case GaitType::TRIPOD:
        default: {
            // Tripod: 3 legs in air at a time (50.0% swing, 50.0% stance)
            swingRatio = 0.5f;
            const float tripodOffsets[LEG_COUNT] = { 0.0f, 0.5f, 0.5f, 0.0f, 0.0f, 0.5f };
            for (int k = 0; k < LEG_COUNT; k++) offsets[k] = tripodOffsets[k];
            break;
        }
    }

    float stanceRatio = 1.0f - swingRatio;

    for (int i = 0; i < LEG_COUNT; i++) {
        float legPhase = m_phaseClock + offsets[i];
        if (legPhase >= 1.0f) legPhase -= 1.0f;

        // Apply dynamic legStance (outward reach) and hipStance (splay angle)
        float baseFootX = DEFAULT_FOOT_X + cmd.legStance;
        float baseFootY = DEFAULT_FOOT_Y;

        if (fabsf(cmd.hipStance) > 0.01f) {
            float hipRad = cmd.hipStance * (M_PI / 180.0f);
            float splay = (i == 0 || i == 5) ? hipRad : ((i == 2 || i == 3) ? -hipRad : 0.0f);
            float cosH = cosf(splay), sinH = sinf(splay);
            float rx = baseFootX * cosH - baseFootY * sinH;
            float ry = baseFootX * sinH + baseFootY * cosH;
            baseFootX = rx;
            baseFootY = ry;
        }

        // 1. Base translation velocities
        float strideX = cmd.vx * (cmd.cycleTime * stanceRatio);
        float strideY = cmd.vy * (cmd.cycleTime * stanceRatio);

        // 2. Add rotational (tangential) velocities based on omega
        if (fabsf(cmd.omega) > 0.1f) {
            float yawRad = (cmd.omega * (M_PI / 180.0f)) * (cmd.cycleTime * stanceRatio);
            
            // Approximate coordinates of the leg relative to the body center (to calculate pivot leverage)
            float bodyOffsetX = (i == 0 || i == 5) ? 1.0f : ((i == 2 || i == 3) ? -1.0f : 0.0f); // Front (+1), Middle (0), Rear (-1)
            float bodyOffsetY = (i < 3) ? -1.0f : 1.0f; // Right Side (-1), Left Side (+1)

            // Tangential cross-product to convert rotation into X/Y foot travel
            float rotStrideX = -bodyOffsetY * yawRad * (DEFAULT_FOOT_X + cmd.legStance);
            float rotStrideY =  bodyOffsetX * yawRad * (DEFAULT_FOOT_X + cmd.legStance);

            strideX += rotStrideX;
            strideY += rotStrideY;
        }

        // 3. Apply final stride vectors to Swing or Stance phase
        if (legPhase < swingRatio) {
            // ── SWING PHASE (Air Arc) ─────────────────────────────────────
            float swingProgress = legPhase / swingRatio;
            outputFootTargets[i].x = baseFootX + (-strideX + 2.0f * strideX * swingProgress);
            outputFootTargets[i].y = baseFootY + (-strideY + 2.0f * strideY * swingProgress);
            outputFootTargets[i].z = DEFAULT_FOOT_Z + (sinf(swingProgress * M_PI) * cmd.stepHeight);
        } else {
            // ── STANCE PHASE (Ground Push) ────────────────────────────────
            float stanceProgress = (legPhase - swingRatio) / stanceRatio;
            outputFootTargets[i].x = baseFootX + (strideX - 2.0f * strideX * stanceProgress);
            outputFootTargets[i].y = baseFootY + (strideY - 2.0f * strideY * stanceProgress);
            outputFootTargets[i].z = DEFAULT_FOOT_Z;
        }
    }
}