#pragma once
#include <Arduino.h>
#include <stdint.h>

// ─────────────────────────────────────────────────────────────────────────────
// led_multi_io.h — Multi-LED control on bare GPIO pins via LEDC PWM.
//
// Three runtime modes (switch via MQTT or call led_multi_set_mode()):
//
//   AUTO   Boot-scans candidate pins (CFG_GPIO_LED_PINS).  Any pin reading
//          above CFG_GPIO_LED_DETECT_MV when briefly set as ADC input is
//          assumed to have an LED-to-VCC circuit and added to the active list.
//          All active pins run the built-in breathe pattern automatically.
//          Periodic re-scan keeps the list fresh.
//
//   MANUAL Auto-scan and auto-pattern are disabled.  Each pin is driven
//          exclusively by explicit MQTT commands.  Newly detected pins are
//          still accepted but not auto-driven.
//
//   PCA    GPIO LEDs are all forced off and locked.  Control is yielded to
//          the PCA9685 / led_pca module.  led_multi_handle() becomes a no-op.
//
// MQTT sub-commands (arrived at led_multi_cmd as everything after "led:multi:"):
//
//   mode:auto              — enter AUTO mode (re-scans immediately)
//   mode:manual            — enter MANUAL mode
//   mode:pca               — enter PCA mode (GPIO LEDs off)
//   scan                   — trigger immediate pin re-scan (AUTO/MANUAL)
//   set:<pin>:<duty 0-255> — set one pin (MANUAL only)
//   all:<duty 0-255>       — set all active pins (MANUAL only)
//   enable:<pin>           — force-add a pin to the active list
//   disable:<pin>          — remove a pin from the active list
//   status                 — log detected pins + current mode
// ─────────────────────────────────────────────────────────────────────────────

enum class LedMultiMode : uint8_t
{
    AUTO   = 0,
    MANUAL = 1,
    PCA    = 2,
};

void led_multi_init();

// Call from loop() — drives auto-pattern and periodic re-scan in AUTO mode.
void led_multi_handle();

void        led_multi_set_mode(LedMultiMode mode);
LedMultiMode led_multi_get_mode();

// Scan candidate pins and update the active list.
void led_multi_scan();

// Direct duty control (0-255). Only writes hardware in MANUAL mode;
// silently ignored in AUTO and PCA.
void led_multi_set(uint8_t pin, uint8_t duty);
void led_multi_all(uint8_t duty);

// Force-add / remove a specific pin regardless of scan result.
void led_multi_enable(uint8_t pin);
void led_multi_disable(uint8_t pin);

// Internal dispatcher — called by led.cpp with the sub-message stripped of
// the "led:multi:" prefix.  E.g. "mode:auto" | "scan" | "set:12:128"
void led_multi_cmd(const String &msg);


// Fade multi
void led_multi_fade(uint8_t pin, uint8_t target, uint32_t ms);
void led_multi_fade_all(uint8_t target, uint32_t ms);

// ── Sequence task ─────────────────────────────────────────────────────────
// WLED-inspired: each step has a fade transition + hold duration.
// Runs as a self-contained FreeRTOS task — no MQTT involvement after start.
#define LED_MULTI_SEQ_MAX_STEPS 16

struct LedMultiSeqStep
{
    uint8_t pin;
    uint8_t duty;
    uint32_t fadems; // transition duration
    uint32_t holdms; // time to hold at target before next step
};

void led_multi_seq_start(const LedMultiSeqStep *steps, uint8_t count);
void led_multi_seq_stop();
bool led_multi_seq_active();