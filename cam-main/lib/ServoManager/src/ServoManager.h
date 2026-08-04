#pragma once
#include <Arduino.h>

class ServoManager {
public:
    ServoManager();
    void begin();
    
    // Low-level register write:
    void setPWM(uint8_t globalChannel, uint16_t onTick, uint16_t offTick);

    // High-level phase-staggered writes:
    void setServoWidthTicks(uint8_t globalChannel, uint16_t widthTicks);
    void setServoPulseUs(uint8_t globalChannel, uint16_t pulseUs);
    
    void setOutputsEnabled(bool enabled);
    bool isOutputsEnabled() const { return m_outputsEnabled; }
    
private:
    uint8_t m_boardAddresses[2];
    bool m_boardActive[2]; // Tracks if hardware actually responded
    bool m_outputsEnabled; 
    
    bool initBoard(uint8_t boardIndex);
    bool writeRegister(uint8_t boardAddr, uint8_t reg, uint8_t value);
};