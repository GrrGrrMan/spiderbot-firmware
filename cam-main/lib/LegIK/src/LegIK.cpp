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
    // If it asks for an impossible stretch, CLAMP it to the max physical reach so the leg just fully extends
    float maxReach = (m_l2 + m_l3) - 0.1f;
    float minReach = fabsf(m_l2 - m_l3) + 0.1f;
    
    if (D > maxReach) D = maxReach;
    if (D < minReach) D = minReach;
    if (D == 0.0f) D = 0.1f;

    // 4. Calculate Femur Angle using Law of Cosines
    float alpha1 = atan2f(-targetZ, planarDist); // Angle of D below horizontal
    float cosAlpha2 = (m_l2 * m_l2 + D * D - m_l3 * m_l3) / (2.0f * m_l2 * D);
    cosAlpha2 = constrain(cosAlpha2, -1.0f, 1.0f); // Numerical safety clamp
    float alpha2 = acosf(cosAlpha2);
    
    float femurDeg = (alpha1 - alpha2) * (180.0f / M_PI);

    // 5. Calculate Tibia Angle using Law of Cosines
    float cosBeta = (m_l2 * m_l2 + m_l3 * m_l3 - D * D) / (2.0f * m_l2 * m_l3);
    cosBeta = constrain(cosBeta, -1.0f, 1.0f); // Numerical safety clamp
    float beta = acosf(cosBeta);

    // Knee deflection relative to femur axis (neutral extended = 0 deg)
    float tibiaDeg = (M_PI - beta) * (180.0f / M_PI);
    tibiaDeg = tibiaDeg - 90.0f; // Align to mathematical zero

    // 6. ENFORCE HARDWARE SAFETY BOUNDS VIA CLAMPING (Do not abort!)
    coxaDeg  = constrain(coxaDeg,  COXA_MIN_DEG,  COXA_MAX_DEG);
    femurDeg = constrain(femurDeg, FEMUR_MIN_DEG, FEMUR_MAX_DEG);
    tibiaDeg = constrain(tibiaDeg, TIBIA_MIN_DEG, TIBIA_MAX_DEG);

    // Return solved valid joint angles
    result.coxaDeg = coxaDeg;
    result.femurDeg = femurDeg;
    result.tibiaDeg = tibiaDeg;
    result.isValid = true; // Always true, need to fix.

    return result;
}