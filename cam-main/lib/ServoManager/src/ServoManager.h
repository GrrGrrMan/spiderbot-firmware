#pragma once
#include <Arduino.h>

class ServoManager {
public:
    ServoManager();
    void begin();
    void setPWM(uint8_t globalChannel, uint16_t onTick, uint16_t offTick);
    void setOutputsEnabled(bool enabled);
    
private:
    uint8_t m_boardAddresses[2];
    bool m_boardActive[2]; // Tracks if hardware actually responded
    
    bool initBoard(uint8_t boardIndex);
    bool writeRegister(uint8_t boardAddr, uint8_t reg, uint8_t value);
};