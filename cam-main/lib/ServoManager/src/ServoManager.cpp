#include "ServoManager.h"
#include "servo_config.h"
#include "logger.h"
#include <Wire.h>

ServoManager::ServoManager() {
    m_boardAddresses[0] = PCA_ADDR_BOARD_1;
    m_boardAddresses[1] = PCA_ADDR_BOARD_2;
    m_boardActive[0] = false;
    m_boardActive[1] = false;
}

void ServoManager::begin() {
    LOG_SYS("Initializing Dual PCA9685 Servo Drivers...");

    pinMode(PIN_PCA_OE, OUTPUT);
    digitalWrite(PIN_PCA_OE, HIGH); 

    Wire.begin(PIN_PCA_SDA, PIN_PCA_SCL, 400000); 
    
    for (int i = 0; i < PCA_NUM_BOARDS; i++) {
        m_boardActive[i] = initBoard(i);
    }
    delayMicroseconds(500); 

    for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) {
        uint16_t onTick = ch * STAGGER_OFFSET;
        uint16_t offTick = (onTick + SERVO_HOME_TICK) % 4096;
        setPWM(ch, onTick, offTick);
    }
    LOG_MOT("Buffered %d channels with phase-staggered home positions.", NUM_SERVOS);

    digitalWrite(PIN_PCA_OE, LOW);
    LOG_SYS("Dual Servo outputs ENABLED.");
}

bool ServoManager::initBoard(uint8_t boardIndex) {
    uint8_t addr = m_boardAddresses[boardIndex];
    
    // First write tests if the board exists
    if (!writeRegister(addr, PCA_MODE1, 0x11)) {
        LOG_ERR("PCA9685 at 0x%02X NOT FOUND! Check wiring or A0 jumper.", addr);
        return false;
    }
    
    writeRegister(addr, PCA_PRESCALE, 121);
    writeRegister(addr, PCA_MODE1, 0xA1);
    LOG_SYS("PCA9685 at 0x%02X initialized and verified.", addr);
    return true;
}

void ServoManager::setPWM(uint8_t globalChannel, uint16_t onTick, uint16_t offTick) {
    uint8_t boardIndex = globalChannel / 16;
    uint8_t localChannel = globalChannel % 16;

    // Abort if board index is out of bounds or hardware was not found
    if (boardIndex >= PCA_NUM_BOARDS || !m_boardActive[boardIndex]) return; 

    uint8_t boardAddr = m_boardAddresses[boardIndex];
    
    Wire.beginTransmission(boardAddr);
    Wire.write(PCA_LED0_ON_L + 4 * localChannel);
    Wire.write(onTick & 0xFF);
    Wire.write(onTick >> 8);
    Wire.write(offTick & 0xFF);
    Wire.write(offTick >> 8);
    Wire.endTransmission(); // Fire and forget for speed during main loop
}

bool ServoManager::writeRegister(uint8_t boardAddr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(boardAddr);
    Wire.write(reg);
    Wire.write(value);
    uint8_t error = Wire.endTransmission();
    return (error == 0); // 0 means Success (hardware ACK received)
}