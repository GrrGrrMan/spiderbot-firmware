#pragma once

// Optional shared private overrides. This file is gitignored.
#if __has_include("local_config.h")
#include "local_config.h"
#endif

// ─────────────────────────────────────────────────────────────────────────────
// esp_s3.h — self-contained target configuration.
// Hardware pins, intervals, topic strings, and limits resolve in this target profile.
// Never put bare literals in .cpp files — reference a CFG_ constant instead.
// ─────────────────────────────────────────────────────────────────────────────

// ── LED / LEDC ───────────────────────────────────────────────────────────────
#define CFG_LED_PIN 2
#define CFG_LED_FREQ_HZ 5000
#define CFG_LED_RESOLUTION 8 // bits  → 0-255 duty range
#define CFG_LED_TIMER 0      // LEDC_TIMER_0
#define CFG_LED_CHANNEL 0    // LEDC_CHANNEL_0
#define CFG_LED_STOP_FADE_MS 500
#define CFG_LED_LOOP_MAX_CMDS 16
#define CFG_LED_LOOP_CMD_LEN 32
#define CFG_RGB_LED_COUNT 4
#define CFG_RGB_START_CHANNEL 0    // PCA9685 ch 0-11 used for LEDs
#define CFG_RGB_COMMON_ANODE false // true = invert PWM duty
#define CFG_RGB_FADE_STEPS 50      // smoothness of software fades

// ── Single LED — software fade engine ────────────────────────────────────────
#define CFG_LED_FADE_TICK_MS 10 // engine tick rate (~100 Hz)
#define CFG_LED_TIMER_MAX 4     // max concurrent one-shot timers

// ── GPIO Multi-LED ───────────────────────────────────────────────────────────
#define CFG_GPIO_LED_PINS {35}
#define CFG_GPIO_LED_MAX_PINS 8
#define CFG_GPIO_LED_START_CHANNEL 1
#define CFG_GPIO_LED_LEDC_TIMER 0
#define CFG_GPIO_LED_FREQ_HZ 5000
#define CFG_GPIO_LED_DETECT_MV 1800
#define CFG_GPIO_LED_SCAN_INTERVAL_MS 86400000UL
#define CFG_GPIO_LED_BREATHE_STEP_MS 20
#define CFG_GPIO_LED_BREATHE_INC 0.05f

// ── Servo / PCA9685 ──────────────────────────────────────────────────────────
#define CFG_I2C_SDA 15
#define CFG_I2C_SCL 16
#define CFG_PCA9685_ADDR 0x40
#define CFG_PCA9685_SECONDARY_ADDR 0x41
#define CFG_PCA9685_BOARD_COUNT 2
#define CFG_SERVO_FREQ_HZ 50
#define CFG_I2C_FREQ_HZ 400000UL  // 400 kHz for max S3 throughput
#define CFG_SERVO_MIN 100
#define CFG_SERVO_MAX 490 // Safe
#define CFG_SERVO_CHANNELS 28
// Logical channels 0-8 are the left PCA at 0x40.
// Logical channels 9-17 are the right PCA at 0x41, mapped to local 0-8.
#define CFG_SERVO_SECONDARY_CHANNEL_BASE 16
#define CFG_SERVO_MAX_SPEED_DPS 0.0f // V2 honors explicit per-step durations.
#define CFG_SERVO_MOTION_TICK_MS 20UL
#define CFG_SERVO_REPROBE_MS 1000UL

// Freewheel: PCA9685 full-off (no pulse) releases most hobby servos.
// On some servo types "no pulse" causes slow drift; that is expected and safe.
// Always send servo:free before physically rotating a servo to avoid back-EMF.
#define CFG_SERVO_FREE_PULSE 0 // 0 = no PWM output → servo freewheels

// ── Motor V2 RAM sequence layer ──────────────────────────────────────────────
#define CFG_MOTOR_V2_MAX_PROGRAMS 8
#define CFG_MOTOR_V2_MAX_STEPS 48
#define CFG_MOTOR_V2_MAX_NAME_LEN 15
#define CFG_MOTOR_V2_MAX_TOKENS 24
#define CFG_MOTOR_V2_MIN_ANGLE 0
#define CFG_MOTOR_V2_MAX_ANGLE 180
#define CFG_MOTOR_V2_MAX_MOVE_MS 120000UL
#define CFG_MOTOR_V2_MAX_WAIT_MS 300000UL
#define CFG_MOTOR_V2_MAX_LOOPS 10000UL
#define CFG_MOTOR_V2_DEFAULT_MOVE_MS 0UL
#define CFG_MOTOR_V2_ENFORCE_SPEED_LIMIT 0
#define CFG_MOTOR_V2_POSE_CHANNEL_STAGGER_MS 8UL

// ── WiFi / network recovery ─────────────────────────────────────────────────
#define CFG_WIFI_CONNECT_ATTEMPTS 20
#define CFG_WIFI_RETRY_MS 30000UL
#define CFG_WIFI_SCAN_WHEN_IDLE_MS 10000UL
#define CFG_WIFI_HOTSPOT_SSID "spiderlink"
#define CFG_WIFI_HOTSPOT_PREFER_SCAN_MS 60000UL
#define CFG_WIFI_HOTSPOT_FAILED_BACKOFF_MS 300000UL
#define CFG_WIFI_ENABLE_ENTERPRISE 0

// ── Target identity / board defaults ────────────────────────────────────────
// ESP32-S3 IPEX target profile. This board uses the v2 topic root directly
// and has a bright onboard WS2812 on IO48 that can be disabled at boot.
#define CFG_TARGET_NAME "esp_s3"
#define CFG_DEFAULT_MQTT_CLIENT "esp32s3-spiderbot"
#define CFG_DEFAULT_MQTT_NAMESPACE "alphaesp32s3"
#define CFG_DEFAULT_MQTT_DEVICE_ID "spiderbot-s3"
#define CFG_DEFAULT_MQTT_TOPIC_OTA CFG_MQTT_TOPIC_ROOT "/ota"
#define CFG_DEFAULT_MQTT_TOPIC_CMD CFG_MQTT_TOPIC_ROOT "/cmd"
#define CFG_DEFAULT_MQTT_TOPIC_LOG CFG_MQTT_TOPIC_ROOT "/log"
#define CFG_DEFAULT_OTA_BRANCH "main"
#define CFG_DEFAULT_OTA_FALLBACK_BRANCH "main"
#define CFG_DEFAULT_OTA_ARTIFACT_BASENAME "bin/firmware-esp32s3"
#define CFG_DEFAULT_ONBOARD_RGB_PIN 48
#if defined(FEATURE_ONBOARD_RGB_OFF)
#define CFG_DEFAULT_ONBOARD_RGB_DISABLE 1
#else
#define CFG_DEFAULT_ONBOARD_RGB_DISABLE 0
#endif

#if __has_include("local_config.esp_s3.h")
#include "local_config.esp_s3.h"
#endif
// ── Board-level built-in RGB cleanup ────────────────────────────────────────
// FEATURE_ONBOARD_RGB_OFF disables the Lonely Binary ESP32-S3 IPEX's bright
// built-in WS2812 on IO48. This is separate from FEATURE_RGB, which controls
// external PCA9685 RGB LEDs.
#ifndef CFG_ONBOARD_RGB_PIN
#define CFG_ONBOARD_RGB_PIN CFG_DEFAULT_ONBOARD_RGB_PIN
#endif
#ifndef CFG_ONBOARD_RGB_DISABLE
#define CFG_ONBOARD_RGB_DISABLE CFG_DEFAULT_ONBOARD_RGB_DISABLE
#endif
#ifndef CFG_ONBOARD_RGB_OFF_REPEAT_MS
#define CFG_ONBOARD_RGB_OFF_REPEAT_MS 250UL
#endif
#ifndef CFG_ONBOARD_RGB_OFF_REPEAT_COUNT
#define CFG_ONBOARD_RGB_OFF_REPEAT_COUNT 12
#endif

// ── MQTT ─────────────────────────────────────────────────────────────────────
// Local-only broker policy:
//
// 1. If connected to the spiderlink hotspot, use the hotspot gateway broker.
// 2. On normal WiFi, do not probe public brokers or Pi hostnames. Open the
//    local control/link port and wait for a signed LAN link from a Pi.
// 3. If no whitelisted WiFi is visible, stay in WiFi scan standby.

#ifndef CFG_MQTT_HOST
#define CFG_MQTT_HOST ""
#endif
#ifndef CFG_MQTT_PORT
#define CFG_MQTT_PORT 1883
#endif
#ifndef CFG_MQTT_PRIMARY_HOST
#define CFG_MQTT_PRIMARY_HOST "192.168.4.1"
#endif
#ifndef CFG_MQTT_PRIMARY_PORT
#define CFG_MQTT_PRIMARY_PORT 1883
#endif
#ifndef CFG_MQTT_LOCAL_BROKERS
#define CFG_MQTT_LOCAL_BROKERS                                      \
    {                                                               \
        {"hotspot", CFG_MQTT_PRIMARY_HOST, CFG_MQTT_PRIMARY_PORT}, \
    }
#endif
#ifndef CFG_MQTT_LOCAL_BROKER_PROBE_MS
#define CFG_MQTT_LOCAL_BROKER_PROBE_MS 800UL
#endif
#ifndef CFG_MQTT_FAILOVER_MS
#define CFG_MQTT_FAILOVER_MS 10000UL
#endif
#ifndef CFG_MQTT_PRIMARY_RECHECK_MS
#define CFG_MQTT_PRIMARY_RECHECK_MS 300000UL
#endif
#ifndef CFG_MQTT_LOCAL_RECHECK_MS
#define CFG_MQTT_LOCAL_RECHECK_MS 30000UL
#endif
#ifndef CFG_LOCAL_CONTROL_PORT
#define CFG_LOCAL_CONTROL_PORT 7777
#endif
#ifndef CFG_LOCAL_CONTROL_MAX_LINE
#define CFG_LOCAL_CONTROL_MAX_LINE 512
#endif
#ifndef CFG_MQTT_CLIENT
#define CFG_MQTT_CLIENT CFG_DEFAULT_MQTT_CLIENT
#endif
#ifndef CFG_MQTT_NAMESPACE
#define CFG_MQTT_NAMESPACE CFG_DEFAULT_MQTT_NAMESPACE
#endif
#ifndef CFG_MQTT_DEVICE_ID
#define CFG_MQTT_DEVICE_ID CFG_DEFAULT_MQTT_DEVICE_ID
#endif
#ifndef CFG_MQTT_TOPIC_ROOT
#define CFG_MQTT_TOPIC_ROOT CFG_MQTT_NAMESPACE "/" CFG_MQTT_DEVICE_ID
#endif
#ifndef CFG_MQTT_TOPIC_OTA
#define CFG_MQTT_TOPIC_OTA CFG_DEFAULT_MQTT_TOPIC_OTA
#endif
#ifndef CFG_MQTT_TOPIC_CMD
#define CFG_MQTT_TOPIC_CMD CFG_DEFAULT_MQTT_TOPIC_CMD
#endif
#ifndef CFG_MQTT_TOPIC_LOG
#define CFG_MQTT_TOPIC_LOG CFG_DEFAULT_MQTT_TOPIC_LOG
#endif
#ifndef CFG_MQTT_TOPIC_CMD_DISCRETE
#define CFG_MQTT_TOPIC_CMD_DISCRETE CFG_MQTT_TOPIC_ROOT "/cmd/discrete"
#endif
#ifndef CFG_MQTT_TOPIC_CMD_MOTION
#define CFG_MQTT_TOPIC_CMD_MOTION CFG_MQTT_TOPIC_ROOT "/cmd/motion"
#endif
#ifndef CFG_MQTT_TOPIC_CMD_MOTOR
#define CFG_MQTT_TOPIC_CMD_MOTOR CFG_MQTT_TOPIC_ROOT "/cmd/motor"
#endif
#ifndef CFG_MQTT_TOPIC_CONTROLLER_HEARTBEAT
#define CFG_MQTT_TOPIC_CONTROLLER_HEARTBEAT CFG_MQTT_TOPIC_ROOT "/controller/heartbeat"
#endif
#ifndef CFG_MQTT_TOPIC_STATE
#define CFG_MQTT_TOPIC_STATE CFG_MQTT_TOPIC_ROOT "/state"
#endif
#ifndef CFG_MQTT_TOPIC_EVENT
#define CFG_MQTT_TOPIC_EVENT CFG_MQTT_TOPIC_ROOT "/event"
#endif
#ifndef CFG_MQTT_TOPIC_LOG_V2
#define CFG_MQTT_TOPIC_LOG_V2 CFG_MQTT_TOPIC_ROOT "/log"
#endif
#ifndef CFG_MQTT_TOPIC_MOTOR_STATE
#define CFG_MQTT_TOPIC_MOTOR_STATE CFG_MQTT_TOPIC_ROOT "/motor/state"
#endif
#define CFG_MQTT_MAX_MSG 512
// Buffer doubled from 512 → 1024 to absorb group-send bursts (16 × ~40B msgs).
// The previous 512B limit caused mid-burst truncation and board crashes.
#define CFG_MQTT_BUFFER_SIZE 2048
#define CFG_MQTT_RETRY_MS 10000
#define CFG_MQTT_KEEPALIVE_SEC 30
#define CFG_CONTROL_HEARTBEAT_TIMEOUT_MS 4000UL
#define CFG_CONTROL_STATE_PUBLISH_MS 1000UL

// ── Ultrasound Sensor (HC-SR04) ──────────────────────────────────────────────
#define CFG_ULTRASOUND_CONFLICT_NOTE 1
#define CFG_ULTRASOUND_TRIG_PIN 13
#define CFG_ULTRASOUND_ECHO_PIN 14
#define CFG_ULTRASOUND_LED_PIN 35
#define CFG_ULTRASOUND_MIN_CM 2.0f
#define CFG_ULTRASOUND_MAX_CM 100.0f
#define CFG_ULTRASOUND_SAMPLE_MS 100
#define CFG_ULTRASOUND_TIMEOUT_US 25000UL
#define CFG_ULTRASOUND_TOPIC_DIST "beanspiderbot/sensor/distance"
#define CFG_ULTRASOUND_TOPIC_STATUS "beanspiderbot/sensor/status"

// ── Logical lights — WLED-like state, current hardware ─────────────────────
// PCA9685 LED output shares the servo PCA frequency. 50 Hz is acceptable for
// test LEDs, but a dedicated LED-only PCA should use a higher frequency.
#define CFG_LIGHT_TOPIC_PREFIX "spiderbot/s3/lights"
#define CFG_LIGHT_HEALTH_TOPIC "spiderbot/s3/health"
#define CFG_LIGHT_CAPABILITIES_TOPIC "spiderbot/s3/capabilities"

// ── Safe Boot ────────────────────────────────────────────────────────────────
#define CFG_SAFEBOOT_THRESHOLD 3
#define CFG_SAFEBOOT_STABLE_MS 30000UL

// ── OTA Pull — Primary (private repo, requires GITHUB_TOKEN in secrets.h) ───
#ifndef CFG_OTA_REPO_OWNER
#define CFG_OTA_REPO_OWNER "GrrGrrMan"
#endif
#ifndef CFG_OTA_REPO_NAME
#define CFG_OTA_REPO_NAME "legacy-hexapod"
#endif
#ifndef CFG_OTA_BRANCH
#define CFG_OTA_BRANCH CFG_DEFAULT_OTA_BRANCH
#endif
#ifndef CFG_OTA_ARTIFACT_BASENAME
#define CFG_OTA_ARTIFACT_BASENAME CFG_DEFAULT_OTA_ARTIFACT_BASENAME
#endif
#define CFG_OTA_CHECK_MS 18000000UL // 5 hours between auto-checks
#define CFG_OTA_MIN_HEAP 60000
#define CFG_OTA_TASK_STACK 20480
#define CFG_OTA_HASH_TIMEOUT 10000
#define CFG_OTA_DL_TIMEOUT 30000

// ── OTA Pull — Fallback (public repo, no auth token required) ───────────────
#ifndef CFG_OTA_FALLBACK_REPO_OWNER
#define CFG_OTA_FALLBACK_REPO_OWNER "GrrGrrMan"
#endif
#ifndef CFG_OTA_FALLBACK_REPO_NAME
#define CFG_OTA_FALLBACK_REPO_NAME "ESP32-SpiderBot-BIN"
#endif
#ifndef CFG_OTA_FALLBACK_BRANCH
#define CFG_OTA_FALLBACK_BRANCH CFG_DEFAULT_OTA_FALLBACK_BRANCH
#endif
