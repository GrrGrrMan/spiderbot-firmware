#pragma once

// Hardware Pins (ESP32-S3-DevKitC-1)
// Shared I2C bus drives both PCA9685 boards (see PLAN.md section 4 pin map)
#define PIN_PCA_SDA 41
#define PIN_PCA_SCL 42
#define PIN_PCA_OE  13

const uint8_t LEG_COXA_CHANNELS[6]  = { 24, 20, 16,  8,  4,  0 }; // RF, RM, RB, LB, LM, LF
const uint8_t LEG_FEMUR_CHANNELS[6] = { 25, 21, 17,  9,  5,  1 };
const uint8_t LEG_TIBIA_CHANNELS[6] = { 26, 22, 18, 10,  6,  2 };


// Dual PCA9685 I2C Addresses
#define PCA_ADDR_BOARD_1 0x40 // Default address (LEFT side)
#define PCA_ADDR_BOARD_2 0x41 // A0 jumper bridged (RIGHT side)
#define PCA_NUM_BOARDS 2

// PCA9685 Registers
#define PCA_MODE1       0x00
#define PCA_PRESCALE    0xFE
#define PCA_LED0_ON_L   0x06

// Hexapod Specifics
#define NUM_SERVOS      18  // 3-DOF * 6 legs
#define SERVO_HOME_TICK 307 // ~1500us at 50Hz (20ms period)
#define STAGGER_OFFSET  227 // 4096 / 18 ≈ 227 ticks apart to prevent power spikes