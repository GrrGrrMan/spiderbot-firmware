#pragma once

#include <Arduino.h>

// Hardware Pins (ESP32-S3-DevKitC-1)
// Shared I2C bus drives both PCA9685 boards (see PLAN.md section 4 pin map)
#define PIN_PCA_SDA 41
#define PIN_PCA_SCL 42
#define PIN_PCA_OE  13

const uint8_t LEG_COXA_CHANNELS[6]  = {  0,  4,  8, 16, 20, 24 }; // RF, RM, RB, LB, LM, LF
const uint8_t LEG_FEMUR_CHANNELS[6] = {  1,  5,  9, 17, 21, 25 };
const uint8_t LEG_TIBIA_CHANNELS[6] = {  2,  6, 10, 18, 22, 26 };

// Dual PCA9685 I2C Addresses
#define PCA_ADDR_BOARD_1 0x40 // Default address (RIGHT side, PCA 0)
#define PCA_ADDR_BOARD_2 0x41 // A0 jumper bridged (LEFT side, PCA 1)
#define PCA_NUM_BOARDS 2

// PCA9685 Registers
#define PCA_MODE1          0x00
#define PCA_MODE2          0x01
#define PCA_PRESCALE       0xFE
#define PCA_LED0_ON_L      0x06
#define PCA_ALL_LED_ON_L   0xFA

// Hexapod Specifics
#define NUM_SERVOS          18  // 3-DOF * 6 legs
#define SERVO_HOME_TICK     307 // ~1500us at 50Hz (20ms period)
#define LOCAL_STAGGER_TICKS 150 // Per-board stagger: max tick 2250 + 490 = 2740 < 4096 (Guaranteed no wrap-around)