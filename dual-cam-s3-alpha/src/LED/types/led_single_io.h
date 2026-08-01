#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
// led_single_io.h — Single onboard LED via ESP32 LEDC PWM.
//
// Owns CFG_LED_PIN on LEDC channel CFG_LED_CHANNEL / timer CFG_LED_TIMER.
// Nothing outside this module writes to that pin or channel directly.
//
// MQTT sub-commands (arrived at led_single_cmd as everything after "led:single:"):
//
//   set:<duty>                               — instant brightness 0-255
//   fade:<target>:<ms>                       — hardware fade (non-blocking)
//   loop:start:<gap_ms>|<cmd1>|<cmd2>|...   — FreeRTOS loop sequence
//   loop:stop                                — stop running loop
//
// Loop inner commands (pipe-separated in the loop payload):
//   set:<duty>        — instant set, then wait gap_ms
//   fade:<t>:<ms>     — hardware fade over ms, then wait gap_ms
//   stop              — fade to 0 over CFG_LED_STOP_FADE_MS, then wait
// ─────────────────────────────────────────────────────────────────────────────

void    led_single_init();
void    led_single_set(uint8_t duty);
void    led_single_fade_to(uint8_t target, uint32_t ms);
// Fade curves (WLED-inspired)
enum class LedFadeCurve : uint8_t
{
    LINEAR = 0,      // t/T * range
    SINE = 1,        // sin(t/T * π/2) — feels most natural
    EXPONENTIAL = 2, // (e^t - 1) / (e - 1) — aggressive top
    LOGARITHMIC = 3, // log(t+1) / log(T+1) — soft ramp
    QUADRATIC = 4,   // (t/T)^2 — gentle start
    INV_QUAD = 5,    // 1 - (1-t/T)^2 — gentle end
};

// Non-blocking software fade (runs via led_single_handle)
void led_single_fade_soft(uint8_t target, uint32_t ms,
                          LedFadeCurve curve = LedFadeCurve::SINE);
bool led_single_fade_active();
void led_single_fade_cancel();

// One-shot timers
void led_single_timer_set(uint8_t duty, uint32_t delay_ms); // set duty after N ms
void led_single_timer_cancel_all();

// Tick — call from led_handle() / loop()
void led_single_handle();

uint8_t led_single_get();

void led_single_loop_start(uint32_t gap_ms, const char *cmds[], uint8_t count);
void led_single_loop_stop();
bool led_single_loop_active();

// Internal dispatcher — called by led.cpp with the sub-message stripped of
// the "led:single:" prefix.  E.g. "set:255" | "fade:128:500" | "loop:stop"
void led_single_cmd(const String &msg);
