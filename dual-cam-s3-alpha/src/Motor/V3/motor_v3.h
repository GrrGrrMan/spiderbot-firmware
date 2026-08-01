#ifndef MOTOR_V3_H
#define MOTOR_V3_H

#include <Arduino.h>
#include "Build/config/target_config.h"

void motor_v3_init();
void motor_v3_handle();
void motor_v3_handle_stream_json(const String &payload);

#endif