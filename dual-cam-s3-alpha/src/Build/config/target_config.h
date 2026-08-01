#pragma once

// Target selector only. Actual CFG_* values live in the per-board configs:
//   src/Build/esp_cam.h
//   src/Build/esp_s3.h
#ifndef ALPHA_TARGET_CONFIG_HEADER
#if defined(ALPHA_TARGET_ESP32S3_IPEX)
#define ALPHA_TARGET_CONFIG_HEADER "Build/esp_s3.h"
#elif defined(ALPHA_TARGET_ESP32CAM)
#define ALPHA_TARGET_CONFIG_HEADER "Build/esp_cam.h"
#else
#error "No AlphaESP target selected. Define ALPHA_TARGET_CONFIG_HEADER in platformio.ini."
#endif
#endif

#include ALPHA_TARGET_CONFIG_HEADER
