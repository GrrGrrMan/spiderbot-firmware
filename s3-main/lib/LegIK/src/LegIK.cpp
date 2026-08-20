#include "LegIK.h"
#include <math.h>

LegIK::LegIK() 
    : m_l1(COXA_LENGTH_MM), 
      m_l2(FEMUR_LENGTH_MM), 
      m_l3(TIBIA_LENGTH_MM) {}

LegAngles LegIK::solveIK(float targetX, float targetY, float targetZ) {
    LegAngles result = {0.0f, 0.0f, 0.0f, false};

    // 1. Coxa (Hip Pan) Angle
    float coxaRad = atan2f(targetY, targetX);
    float coxaDeg = coxaRad * (180.0f / M_PI);

    // 2. Projected 2D Femur-Tibia distance
    float planarDist = sqrtf(targetX * targetX + targetY * targetY) - m_l1;
    float D = sqrtf(planarDist * planarDist + targetZ * targetZ);

    // 3. Reachability Failsafe Clamping
    float maxReach = (m_l2 + m_l3) - 0.1f;
    float minReach = fabsf(m_l2 - m_l3) + 0.1f;
    if (D > maxReach) D = maxReach;
    if (D < minReach) D = minReach;
    if (D == 0.0f) D = 0.1f;

    // 4. Calculate Femur Angle
    float alpha1 = atan2f(-targetZ, planarDist);
    float cosAlpha2 = (m_l2 * m_l2 + D * D - m_l3 * m_l3) / (2.0f * m_l2 * D);
    cosAlpha2 = constrain(cosAlpha2, -1.0f, 1.0f);
    float alpha2 = acosf(cosAlpha2);
    
    float femurDeg = (alpha1 - alpha2) * (180.0f / M_PI);

    // 5. Calculate Tibia Angle
    float cosBeta = (m_l2 * m_l2 + m_l3 * m_l3 - D * D) / (2.0f * m_l2 * m_l3);
    cosBeta = constrain(cosBeta, -1.0f, 1.0f);
    float beta = acosf(cosBeta);

    float tibiaDeg = (M_PI - beta) * (180.0f / M_PI) - 90.0f;

    // 6. Joint Limit Enforcements
    coxaDeg  = constrain(coxaDeg,  COXA_MIN_DEG,  COXA_MAX_DEG);
    femurDeg = constrain(femurDeg, FEMUR_MIN_DEG, FEMUR_MAX_DEG);
    tibiaDeg = constrain(tibiaDeg, TIBIA_MIN_DEG, TIBIA_MAX_DEG);

    result.coxaDeg = coxaDeg;
    result.femurDeg = femurDeg;
    result.tibiaDeg = tibiaDeg;
    result.isValid = true;

    return result;
}