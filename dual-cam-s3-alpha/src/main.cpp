#include <Arduino.h>
#include "Network/network.h"
#include "Network/local_control.h"
#include "Network/mqtt/mqtt_trigger.h"
#include "Build/Log/cmd_registry.h"
#include "Build/Log/logger.h"
#include "Build/secure/safeboot.h"
#include "Build/OTA/ota_pull.h"
#include "LED/types/onboard_rgb.h"

#if defined(FEATURE_LED) || defined(FEATURE_GPIO_LEDS) || defined(FEATURE_RGB)
#define FEATURE_LED_SUBSYSTEM
#endif

// ── LED subsystem (single / multi-GPIO / PCA9685 RGB) ─────────────────────
#ifdef FEATURE_LED_SUBSYSTEM
#include "LED/led.h"
#endif

// ── Other subsystems ──────────────────────────────────────────────────────
#if defined(FEATURE_MOTOR_V3)
#include "Motor/V3/motor_v3.h"
#elif defined(FEATURE_MOTOR_V2)
#include "Motor/V2/motor_v2.h"
#elif defined(FEATURE_SERVO)
#include "Motor/servos.h"
#endif

#ifdef FEATURE_ULTRASOUND
#include "Sensor/ultrasound.h"
#endif

#ifdef FEATURE_LIGHTS
#include "LED/light_control.h"
#endif
// ─────────────────────────────────────────────────────────────────────────────
// Increase the Arduino loop task stack for TLS (required by OTA pull).
// ─────────────────────────────────────────────────────────────────────────────
SET_LOOP_TASK_STACK_SIZE(16 * 1024);

// ─────────────────────────────────────────────────────────────────────────────
// setup()
//
// Init order:
//   1. Serial + safeboot_check()  — reads RTC crash counter; if threshold hit,
//      only WiFi + OTA start and we return early.
//   2. network_init()             — WiFi + ArduinoOTA (push OTA).
//   3. mqtt_trigger_init()        — registers log sink + built-in commands.
//   4. motor_v2_init()/servos_init()
//      MUST precede led_init() when FEATURE_RGB is active: led_pca shares the
//      PCA9685 started by the motor layer.
//   5. led_init()                 — initialises all enabled LED sub-modules.
//      led_register_commands()   — registers single "led:" prefix covering
//                                  led:single:, led:multi:, led:pca: routing.
//   6. ota_pull_init()            — GitHub pull OTA; runs via loop().
//   7. tester_init()              — diagnostics; always last.
//   8. future later
// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    delay(200);
    onboard_rgb_init();

    // ── Crash-loop guard ───────────────────────────────────────────────
    bool safeMode = safeboot_check();
    LOG("Booting..." + String(safeMode ? " [SAFE MODE]" : ""));

    // ── WiFi + ArduinoOTA ─────────────────────────────────────────────
    network_init();

    if (safeMode)
    {
        // Safe mode: WiFi and ArduinoOTA are live.
        // MQTT also starts so "reset" and "ota" commands still work.
#ifdef FEATURE_MQTT
        mqtt_trigger_init();
#endif
        local_control_init();
        LOG("[SAFEBOOT] WiFi + OTA ready. Waiting for new firmware.");
        return;
    }

    // ── MQTT ───────────────────────────────────────────────────────────
#ifdef FEATURE_MQTT
    mqtt_trigger_init();
#endif

    // ── Motor/PCA9685 — must come BEFORE led_init() when FEATURE_RGB active ──
#if defined(FEATURE_MOTOR_V3)
    motor_v3_init();
#elif defined(FEATURE_MOTOR_V2)
    motor_v2_init();
    motor_v2_register_commands();
#elif defined(FEATURE_SERVO)
    servos_init();
    servos_register_commands(); // registers "servo:"
#endif

#ifdef FEATURE_LIGHTS
    light_init();
    light_register_commands(); // registers "light:" and direct light topics
#endif

    // ── LED subsystem — only present when at least one LED flag is enabled ─
#ifdef FEATURE_LED_SUBSYSTEM
    led_init();
    led_register_commands(); // registers "led:" → routes led:single/multi/pca
#endif

#ifdef FEATURE_ULTRASOUND
    ultrasound_init();
    ultrasound_register_commands(); // registers "sensor:" prefix
#endif

    // ── Local TCP control/link port ────────────────────────────────────
    local_control_init();

    // ── Pull OTA ───────────────────────────────────────────────────────
#ifdef FEATURE_OTA_PULL
    ota_pull_init(OtaMode::AUTO);
#endif

    LOG("Ready.");
}

// ─────────────────────────────────────────────────────────────────────────────
// loop()
// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
    onboard_rgb_handle();
    network_handle();
    safeboot_update();

    if (!network_connected())
        return;

    local_control_handle();

#ifdef FEATURE_MQTT
    mqtt_trigger_handle();
#endif

    if (safeboot_active()) return;

#ifdef FEATURE_OTA_PULL
    ota_pull_handle();
#endif

#ifdef FEATURE_LED_SUBSYSTEM
    // Single call drives all enabled LED sub-module loop() work.
    led_handle();
#endif

#ifdef FEATURE_ULTRASOUND
    ultrasound_handle();
#endif

#ifdef FEATURE_LIGHTS
    light_handle();
#endif

#if defined(FEATURE_MOTOR_V3)
    motor_v3_handle();
#elif defined(FEATURE_MOTOR_V2)
    motor_v2_handle();
#elif defined(FEATURE_SERVO)
    servos_handle();
#endif

}
