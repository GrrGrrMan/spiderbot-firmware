#pragma once
#include <Arduino.h>
#include <stdint.h>

// ─────────────────────────────────────────────────────────────────────────────
// gpio_leds.h — Multi-LED control on bare GPIO pins via LEDC PWM.
//
// Three runtime modes (switch via MQTT or call gpio_leds_set_mode()):
//
//   AUTO   Boot-scans candidate pins (CFG_GPIO_LED_PINS).  Any pin that
//          reads above CFG_GPIO_LED_DETECT_MV when briefly set as ADC input
//          is assumed to have an LED-to-VCC circuit and is added to the
//          active list.  All active pins run the built-in breathe pattern
//          automatically.  Periodic re-scan keeps the list fresh.
//
//   MANUAL Auto-scan and auto-pattern are disabled.  Each pin is driven
//          exclusively by explicit MQTT "gpled:set" / "gpled:all" commands.
//          Newly detected pins are still accepted but no auto-driving occurs.
//
//   PCA    GPIO LEDs are all forced off and locked.  Control is yielded to
//          the PCA9685 / rgb module.  gpio_leds_handle() becomes a no-op.
//
// MQTT command protocol (prefix "gpled:"):
//   gpled:mode:auto              — enter AUTO mode (re-scans immediately)
//   gpled:mode:manual            — enter MANUAL mode
//   gpled:mode:pca               — enter PCA mode (GPIO LEDs off)
//   gpled:scan                   — trigger immediate pin re-scan (AUTO/MANUAL)
//   gpled:set:<pin>:<duty 0-255> — set one pin (MANUAL only)
//   gpled:all:<duty 0-255>       — set all active pins (MANUAL only)
//   gpled:enable:<pin>           — force-add a pin to the active list
//   gpled:disable:<pin>          — remove a pin from the active list
//   gpled:status                 — log detected pins + current mode
// ─────────────────────────────────────────────────────────────────────────────

enum class GpioLedMode : uint8_t
{
    AUTO = 0,
    MANUAL = 1,
    PCA = 2,
};

void gpio_leds_init();

// Call from loop() — drives auto-pattern and periodic re-scan in AUTO mode.
void gpio_leds_handle();

void gpio_leds_set_mode(GpioLedMode mode);
GpioLedMode gpio_leds_get_mode();

// Scan candidate pins and update the active list.
void gpio_leds_scan();

// Direct duty control (0-255).  Only writes hardware in MANUAL mode;
// silently ignored in AUTO and PCA.
void gpio_leds_set(uint8_t pin, uint8_t duty);
void gpio_leds_all(uint8_t duty);

// Force-add / remove a specific pin regardless of scan result.
void gpio_leds_enable(uint8_t pin);
void gpio_leds_disable(uint8_t pin);

// Register "gpled:" prefix with cmd_registry.
void gpio_leds_register_commands();