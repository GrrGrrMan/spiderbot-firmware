#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
// led_pca.h — Multi-RGB-LED controller via PCA9685.
//
// Each LED is addressed by index 0..CFG_RGB_LED_COUNT-1.
// Channels are laid out as: base + (id * 3) + {0=R, 1=G, 2=B}.
//
// Requires: servos_init() must have been called first — the PCA9685
// Adafruit_PWMServoDriver instance (pwm) is shared with servos.cpp.
//
// Pattern animation runs as a cooperative state machine ticked by
// led_pca_handle(), which must be called from led_handle() every loop().
// No FreeRTOS tasks are created.
//
// led_pca_fade() is non-blocking — progress is advanced in led_pca_handle().
// Do not run a pattern and a fade on the same LED simultaneously.
//
// MQTT sub-commands (arrived at led_pca_cmd as everything after "led:pca:"):
//
//   set:<id>:<r>:<g>:<b>                    — instant colour
//   fade:<id>:<r>:<g>:<b>:<ms>              — non-blocking software fade
//   all:<r>:<g>:<b>                         — set all LEDs same colour
//   off                                     — all LEDs off
//   pattern:blink:<id>:<r>:<g>:<b>:<on>:<off>
//   pattern:pulse:<id>:<r>:<g>:<b>:<ms>
//   pattern:chase:<r>:<g>:<b>:<gap_ms>
//   pattern:stop                            — stop pattern, all LEDs off
// ─────────────────────────────────────────────────────────────────────────────

void led_pca_init();
void led_pca_handle(); // call from led_handle() every loop() iteration

void led_pca_set(uint8_t id, uint8_t r, uint8_t g, uint8_t b);
void led_pca_fade(uint8_t id, uint8_t r, uint8_t g, uint8_t b, uint32_t ms);
void led_pca_all(uint8_t r, uint8_t g, uint8_t b);
void led_pca_off();

void led_pca_pattern_blink(uint8_t id, uint8_t r, uint8_t g, uint8_t b,
                           uint32_t on_ms, uint32_t off_ms);
void led_pca_pattern_pulse(uint8_t id, uint8_t r, uint8_t g, uint8_t b, uint32_t ms);
void led_pca_pattern_chase(uint8_t r, uint8_t g, uint8_t b, uint32_t gap_ms);
void led_pca_pattern_stop();

void led_pca_cmd(const String &msg);