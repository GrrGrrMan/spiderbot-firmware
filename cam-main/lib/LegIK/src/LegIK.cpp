#include "LegIK.h"
#include <math.h>

LegIK::LegIK() 
    : m_l1(COXA_LENGTH_MM), 
      m_l2(FEMUR_LENGTH_MM), 
      m_l3(TIBIA_LENGTH_MM) {}

LegAngles LegIK::solveIK(float targetX, float targetY, float targetZ) {
    LegAngles result = {0.0f, 0.0f, 0.0f, false};

    // 1. Calculate Coxa (Hip Pan) Angle in XY horizontal plane
    float coxaRad = atan2f(targetY, targetX);
    float coxaDeg = coxaRad * (180.0f / M_PI);

    // 2. Project target point into 2D Femur-Tibia plane
    float planarDist = sqrtf(targetX * targetX + targetY * targetY) - m_l1;
    
    // Total 3D straight-line distance from Femur joint to foot tip
    float D = sqrtf(planarDist * planarDist + targetZ * targetZ);

    // 3. Reachability Failsafe Check (Law of Cosines condition)
    if (D > (m_l2 + m_l3) || D < fabsf(m_l2 - m_l3) || D == 0.0f) {
        return result; // Target point is outside physical reach envelope
    }

    // 4. Calculate Femur Angle using Law of Cosines
    float alpha1 = atan2f(-targetZ, planarDist); // Angle of D below horizontal
    float cosAlpha2 = (m_l2 * m_l2 + D * D - m_l3 * m_l3) / (2.0f * m_l2 * D);
    cosAlpha2 = constrain(cosAlpha2, -1.0f, 1.0f); // Numerical safety clamp
    float alpha2 = acosf(cosAlpha2);
    
    float femurDeg = (alpha1 + alpha2) * (180.0f / M_PI);

    // 5. Calculate Tibia Angle using Law of Cosines
    float cosBeta = (m_l2 * m_l2 + m_l3 * m_l3 - D * D) / (2.0f * m_l2 * m_l3);
    cosBeta = constrain(cosBeta, -1.0f, 1.0f); // Numerical safety clamp
    float beta = acosf(cosBeta);

    // Knee deflection relative to femur axis (neutral extended = 0 deg)
    float tibiaDeg = (M_PI - beta) * (180.0f / M_PI);

    // 6. Enforce Hardware Safety Bounds
    if (coxaDeg  < COXA_MIN_DEG  || coxaDeg  > COXA_MAX_DEG  ||
        femurDeg < FEMUR_MIN_DEG || femurDeg > FEMUR_MAX_DEG ||
        tibiaDeg < TIBIA_MIN_DEG || tibiaDeg > TIBIA_MAX_DEG) {
        return result; // Target exceeds joint rotation limits
    }

    // Return solved valid joint angles
    result.coxaDeg = coxaDeg;
    result.femurDeg = femurDeg;
    result.tibiaDeg = tibiaDeg;
    result.isValid = true;

    return result;
}