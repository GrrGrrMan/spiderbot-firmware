#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
// led.h — LED subsystem coordinator.
//
// This is the single entry-point for all LED hardware.  It initialises,
// handles, and registers commands for whichever sub-modules are compiled in
// via platformio.ini feature flags:
//
//   FEATURE_LED        → led_single_io  (onboard LEDC single LED)
//   FEATURE_GPIO_LEDS  → led_multi_io   (bare GPIO multi-LED, auto-detected)
//   FEATURE_RGB        → led_pca        (PCA9685 RGB LEDs)
//
// All three share a single MQTT prefix: "led:"
//
//   led:single:<subcmd>   — routed to led_single_io
//   led:multi:<subcmd>    — routed to led_multi_io
//   led:pca:<subcmd>      — routed to led_pca
//
// Call order in main.cpp (replacing the old per-module calls):
//
//   led_init();             // after servos_init() if FEATURE_RGB is enabled
//   led_register_commands();
//
// In loop():
//   led_handle();           // drives multi-IO breathe pattern + re-scan
// ─────────────────────────────────────────────────────────────────────────────

void led_init();
void led_handle();
void led_register_commands();
