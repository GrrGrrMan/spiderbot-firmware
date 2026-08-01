#pragma once

#include <Arduino.h>

// Motor V2 is a RAM-resident movement/program layer. It intentionally reuses
// the existing PCA9685 servo driver for low-level PWM ownership while replacing
// the network-facing movement language with autonomous sequence execution.

void motor_v2_init();
void motor_v2_handle();
void motor_v2_register_commands();

// Accepts either a full command such as "motor:load ..." or a bare payload from
// the dedicated MQTT motor topic, such as "load ...".
bool motor_v2_handle_command(const String &payload);

// Compatibility entry points for the existing motion/heartbeat topics.
void motor_v2_handle_motion_json(const String &payload);
void motor_v2_handle_heartbeat_json(const String &payload);
