#pragma once
#include <stdint.h>
#include <Arduino.h>

// servos.h - PCA9685 PWM servo driver.
//
// Hardware is detected at init time; all functions are safe to call even
// when the board is absent (they no-op and return false from servos_found()).
//
// Physical forcing warning
// Rotating a servo by hand while it has an active PWM signal will cause the
// motor to fight back, drawing several hundred mA to over 1A per servo.
// On a shared 5V rail this causes a brownout that resets the ESP32.
//
// Always call servos_free(ch) or servos_free_all() before physically
// repositioning any servo. Use servos_set() or servo:center to re-engage.

void servos_init();
void servos_handle();
void servos_set(uint8_t channel, int angle);
void servos_set_timed(uint8_t channel, int angle, uint32_t durationMs);
void servos_center_all();
bool servos_found();

// Low-level PCA9685 PWM helpers for non-servo users such as LED tests.
// These do not update servo angle state; callers must avoid channels that are
// actively driving motors.
bool pca9685_set_pwm_duty(uint8_t channel, uint16_t duty12);
bool pca9685_set_full_off(uint8_t channel);

// Freewheel (torque release)
// Sets PCA9685 output to 0 (no pulse), which makes most hobby servos freewheel.
// Safe to call before physical manipulation to prevent back-EMF / brownouts.
// Call servos_set() or servos_center_all() to re-engage.
void servos_free(uint8_t channel);
void servos_free_all();

// Returns the last commanded angle for a channel (90 after init, -1 if freed).
int servos_get_angle(uint8_t channel);

// Register "servo:" prefix with cmd_registry.
// Call after servos_init().
//
// Commands added beyond the original servo:<ch>:<angle>:
//   servo:<ch>:<angle>:<ms>     - move one channel over a requested duration
//   servo:free                - freewheel all channels
//   servo:free:<ch>           - freewheel one channel
//   servo:center              - center all channels at 90 deg
//   servo:center:<ch>         - center one channel
//   servo:status              - log all channel angles over MQTT
void servos_register_commands();

// High-level remote-control protocol delivered on dedicated MQTT topics.
// Motion packets use compact JSON such as:
//   {"session":"abcd1234","seq":12,"channel":3,"angle":90}
//   {"session":"abcd1234","seq":12,"channel":3,"angle":90,"duration_ms":750}
//   {"session":"abcd1234","seq":13,"angles":[90,90,...]}
//   {"session":"abcd1234","seq":14,"angles":[90,90,...],"duration_ms":1200}
// Heartbeats refresh the local watchdog:
//   {"session":"abcd1234","seq":42}
void servos_handle_motion_json(const String &payload);
void servos_handle_heartbeat_json(const String &payload);
