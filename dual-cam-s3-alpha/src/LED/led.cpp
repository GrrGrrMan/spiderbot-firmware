#include "led.h"
#include "Build/config/target_config.h"
#include "Build/Log/logger.h"
#include "Build/Log/cmd_registry.h"

// ─────────────────────────────────────────────────────────────────────────────
// led.cpp — LED subsystem coordinator.
//
// Routes "led:<module>:<subcmd>" MQTT messages to the correct sub-module.
// Sub-modules are compiled in based on FEATURE_ flags; this file always
// compiles and gracefully ignores unavailable modules.
//
// Command routing table:
//   led:single:<subcmd>  →  led_single_cmd(subcmd)   FEATURE_LED
//   led:multi:<subcmd>   →  led_multi_cmd(subcmd)    FEATURE_GPIO_LEDS
//   led:pca:<subcmd>     →  led_pca_cmd(subcmd)      FEATURE_RGB
//
// rgb_led.cpp / rgb_led.h have been removed — they were an un-registered
// duplicate of led_pca that compiled dead code into every build.
// ─────────────────────────────────────────────────────────────────────────────

#ifdef FEATURE_LED
#include "LED/types/led_single_io.h"
#endif

#ifdef FEATURE_GPIO_LEDS
#include "LED/types/led_multi_io.h"
#endif

#ifdef FEATURE_RGB
#include "LED/types/led_pca.h"
#endif

#if defined(FEATURE_LED) || defined(FEATURE_GPIO_LEDS) || defined(FEATURE_RGB)
#define FEATURE_ANY_LED_MODULE
#endif

// ── Unified command dispatcher ────────────────────────────────────────────
static void ledRootHandler(const String &msg)
{
#ifndef FEATURE_ANY_LED_MODULE
    return;
#else
    int first = msg.indexOf(':');
    if (first == -1)
    {
        LOG("[LED] Malformed — expected led:<module>:<cmd>");
        return;
    }

    int second = msg.indexOf(':', first + 1);
    String module = (second == -1) ? msg.substring(first + 1)
                                   : msg.substring(first + 1, second);
    String rest = (second == -1) ? String("") : msg.substring(second + 1);

#ifdef FEATURE_LED
    if (module == "single")
    {
        led_single_cmd(rest);
        return;
    }
#endif

#ifdef FEATURE_GPIO_LEDS
    if (module == "multi")
    {
        led_multi_cmd(rest);
        return;
    }
#endif

#ifdef FEATURE_RGB
    if (module == "pca")
    {
        led_pca_cmd(rest);
        return;
    }
#endif

    String avail = "";
#ifdef FEATURE_LED
    avail += "single ";
#endif
#ifdef FEATURE_GPIO_LEDS
    avail += "multi ";
#endif
#ifdef FEATURE_RGB
    avail += "pca";
#endif
    LOG("[LED] Unknown module '" + module + "'. Available: " + avail);
#endif
}

// ── Public API ────────────────────────────────────────────────────────────
void led_init()
{
#ifndef FEATURE_ANY_LED_MODULE
    return;
#else
#ifdef FEATURE_LED
    led_single_init();
#endif

    // NOTE: led_pca_init() relies on the PCA9685 being started by
    // servos_init() in main.cpp — that ordering must be preserved.
#ifdef FEATURE_RGB
    led_pca_init();
#endif

#ifdef FEATURE_GPIO_LEDS
    led_multi_init();
#endif

    LOG("[LED] Coordinator ready"
#ifdef FEATURE_LED
        " | single"
#endif
#ifdef FEATURE_GPIO_LEDS
        " | multi"
#endif
#ifdef FEATURE_RGB
        " | pca"
#endif
    );
#endif
}

void led_handle()
{
#ifdef FEATURE_ANY_LED_MODULE
#ifdef FEATURE_LED
    led_single_handle();
#endif

#ifdef FEATURE_RGB
    // Tick pattern state machine and one-shot fades.
    // Must come before led_multi_handle() — no ordering dependency,
    // but keeping PCA work grouped with its init block aids readability.
    led_pca_handle();
#endif

#ifdef FEATURE_GPIO_LEDS
    led_multi_handle();
#endif
#endif
}

void led_register_commands()
{
#ifdef FEATURE_ANY_LED_MODULE
    cmd_register("led:", ledRootHandler);
#endif
}
