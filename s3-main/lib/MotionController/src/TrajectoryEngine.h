#pragma once
#include <Arduino.h>

enum class EasingType : uint8_t {
    LINEAR = 0,
    EASE_IN_OUT_CUBIC,
    EASE_IN_OUT_QUAD,
    EASE_IN_OUT_SINE,
    MINIMUM_JERK
};

class TrajectoryEngine {
public:
    // Computes normalized progress s(tau) in [0.0, 1.0] from tau in [0.0, 1.0]
    static inline float evaluateEasing(float tau, EasingType easing) {
        tau = constrain(tau, 0.0f, 1.0f);
        switch (easing) {
            case EasingType::EASE_IN_OUT_CUBIC:
                return (tau < 0.5f) 
                    ? (4.0f * tau * tau * tau) 
                    : (1.0f - powf(-2.0f * tau + 2.0f, 3.0f) / 2.0f);
            
            case EasingType::EASE_IN_OUT_QUAD:
                return (tau < 0.5f)
                    ? (2.0f * tau * tau)
                    : (1.0f - powf(-2.0f * tau + 2.0f, 2.0f) / 2.0f);

            case EasingType::EASE_IN_OUT_SINE:
                return -(cosf(M_PI * tau) - 1.0f) / 2.0f;

            case EasingType::MINIMUM_JERK:
                // Quintic polynomial: 10*tau^3 - 15*tau^4 + 6*tau^5 (zero jerk & velocity at boundaries)
                return tau * tau * tau * (tau * (tau * 6.0f - 15.0f) + 10.0f);

            case EasingType::LINEAR:
            default:
                return tau;
        }
    }

    static inline float interpolate(float startVal, float targetVal, float tau, EasingType easing) {
        float progress = evaluateEasing(tau, easing);
        return startVal + (targetVal - startVal) * progress;
    }
};