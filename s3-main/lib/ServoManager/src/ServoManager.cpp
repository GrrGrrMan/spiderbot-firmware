#include "ServoManager.h"
#include "servo_config.h"
#include "logger.h"
#include <Wire.h>

ServoManager::ServoManager() {
    m_boardAddresses[0] = PCA_ADDR_BOARD_1;
    m_boardAddresses[1] = PCA_ADDR_BOARD_2;
    m_boardActive[0] = false;
    m_boardActive[1] = false;
    m_outputsEnabled = false;
    m_i2cMutex = nullptr;
}

void ServoManager::begin() {
    LOG_SYS("Initializing Dual PCA9685 Servo Drivers...");
    
    if (!m_i2cMutex) {
        m_i2cMutex = xSemaphoreCreateMutex();
    }

    pinMode(PIN_PCA_OE, OUTPUT);
    digitalWrite(PIN_PCA_OE, HIGH); // Ensure hardware outputs remain disabled during setup

    Wire.begin(PIN_PCA_SDA, PIN_PCA_SCL, 400000); 
    Wire.setTimeOut(25); // 25ms timeout protects against I2C bus lockup from servo motor noise
    
    for (int i = 0; i < PCA_NUM_BOARDS; i++) {
        m_boardActive[i] = initBoard(i);
    }
    delay(10); 

    // Pre-buffer initial 1500us pulses into all 18 mapped channels
    for (uint8_t leg = 0; leg < 6; leg++) {
        setServoWidthTicks(LEG_COXA_CHANNELS[leg],  SERVO_HOME_TICK);
        setServoWidthTicks(LEG_FEMUR_CHANNELS[leg], SERVO_HOME_TICK);
        setServoWidthTicks(LEG_TIBIA_CHANNELS[leg], SERVO_HOME_TICK);
    }
    LOG_MOT("Buffered %d mapped channels with non-wrapping phase offsets.", NUM_SERVOS);

    m_outputsEnabled = false;
}

bool ServoManager::initBoard(uint8_t boardIndex) {
    uint8_t addr = m_boardAddresses[boardIndex];
    
    // 1. Put PCA9685 to sleep to allow prescaler configuration
    if (!writeRegister(addr, PCA_MODE1, 0x11)) {
        LOG_ERR("PCA9685 at 0x%02X NOT FOUND! Check wiring or A0 jumper.", addr);
        return false;
    }
    
    // 2. Set PWM prescaler for 50Hz (20ms period)
    writeRegister(addr, PCA_PRESCALE, 121);

    // 3. Clear ALL_LED registers to wipe out any residual Full-OFF (Bit 4) flags
    if (m_i2cMutex && xSemaphoreTake(m_i2cMutex, portMAX_DELAY) == pdTRUE) {
        Wire.beginTransmission(addr);
        Wire.write(PCA_ALL_LED_ON_L);
        Wire.write(0x00); // ALL_LED_ON_L
        Wire.write(0x00); // ALL_LED_ON_H
        Wire.write(0x00); // ALL_LED_OFF_L
        Wire.write(0x00); // ALL_LED_OFF_H (Clears Bit 4 Full-OFF!)
        Wire.endTransmission();
        xSemaphoreGive(m_i2cMutex);
    }
    
    // 4. Wake PCA9685 with Auto-Increment (AI) enabled
    writeRegister(addr, PCA_MODE1, 0xA1);

    // 5. Configure MODE2: Totem-pole (push-pull), update registers on STOP command
    writeRegister(addr, PCA_MODE2, 0x04);

    delayMicroseconds(1000); // Allow internal 25MHz oscillator to stabilize
    LOG_SYS("PCA9685 at 0x%02X initialized and verified.", addr);
    return true;
}

void ServoManager::setPWM(uint8_t globalChannel, uint16_t onTick, uint16_t offTick) {
    uint8_t boardIndex = globalChannel / 16;
    uint8_t localChannel = globalChannel % 16;

    if (boardIndex >= PCA_NUM_BOARDS || !m_boardActive[boardIndex]) return; 

    uint8_t boardAddr = m_boardAddresses[boardIndex];
    
    if (m_i2cMutex && xSemaphoreTake(m_i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        Wire.beginTransmission(boardAddr);
        Wire.write(PCA_LED0_ON_L + 4 * localChannel);
        Wire.write(onTick & 0xFF);
        Wire.write((onTick >> 8) & 0x0F);
        Wire.write(offTick & 0xFF);
        Wire.write((offTick >> 8) & 0x0F);
        Wire.endTransmission();
        xSemaphoreGive(m_i2cMutex);
    }
}

bool ServoManager::writeRegister(uint8_t boardAddr, uint8_t reg, uint8_t value) {
    bool success = false;
    if (m_i2cMutex && xSemaphoreTake(m_i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        Wire.beginTransmission(boardAddr);
        Wire.write(reg);
        Wire.write(value);
        success = (Wire.endTransmission() == 0);
        xSemaphoreGive(m_i2cMutex);
    }
    return success;
}

void ServoManager::setOutputsEnabled(bool enabled) {
    if (m_outputsEnabled == enabled) return; // Prevent redundant calls
    m_outputsEnabled = enabled;

    if (!enabled) {
        // Broadcast Full-OFF to both boards
        if (m_i2cMutex && xSemaphoreTake(m_i2cMutex, portMAX_DELAY) == pdTRUE) {
            for (int i = 0; i < PCA_NUM_BOARDS; i++) {
                if (m_boardActive[i]) {
                    uint8_t addr = m_boardAddresses[i];
                    Wire.beginTransmission(addr);
                    Wire.write(PCA_ALL_LED_ON_L);
                    Wire.write(0x00);
                    Wire.write(0x00);
                    Wire.write(0x00);
                    Wire.write(0x10); // Bit 4 = 1 (Full OFF)
                    Wire.endTransmission();
                }
            }
            xSemaphoreGive(m_i2cMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // Allow in-flight frame to complete
        digitalWrite(PIN_PCA_OE, HIGH);
        LOG_SYS("Hardware Servo Outputs DISABLED (LIMP)");
    } else {
        // Clear Full-OFF broadcast before driving OE low
        if (m_i2cMutex && xSemaphoreTake(m_i2cMutex, portMAX_DELAY) == pdTRUE) {
            for (int i = 0; i < PCA_NUM_BOARDS; i++) {
                if (m_boardActive[i]) {
                    uint8_t addr = m_boardAddresses[i];
                    Wire.beginTransmission(addr);
                    Wire.write(PCA_ALL_LED_ON_L);
                    Wire.write(0x00);
                    Wire.write(0x00);
                    Wire.write(0x00);
                    Wire.write(0x00);
                    Wire.endTransmission();
                }
            }
            xSemaphoreGive(m_i2cMutex);
        }
        digitalWrite(PIN_PCA_OE, LOW);
        LOG_SYS("Hardware Servo Outputs ENABLED");
    }
}

void ServoManager::setServoWidthTicks(uint8_t globalChannel, uint16_t widthTicks) {
    uint8_t localChannel = globalChannel % 16;
    // Per-board channel staggering prevents simultaneous current spikes without wrapping 4096
    uint16_t onTick = localChannel * LOCAL_STAGGER_TICKS;
    uint16_t offTick = onTick + widthTicks;
    setPWM(globalChannel, onTick, offTick);
}

void ServoManager::setServoPulseUs(uint8_t globalChannel, uint16_t pulseUs) {
    uint16_t safePulse = constrain(pulseUs, (uint16_t)488, (uint16_t)2393);
    uint16_t widthTicks = (safePulse * 4096UL) / 20000UL;
    setServoWidthTicks(globalChannel, widthTicks);
}