#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
// ultrasound.h — HC-SR04 distance sensor with LED brightness mapping.
//
// Runs a non-blocking FreeRTOS task on core 1 (away from WiFi) for reliable
// pulseIn() timing without disrupting MQTT/WiFi on core 0.
//
// Optionally links distance → led_multi_io brightness (requires FEATURE_GPIO_LEDS).
// When link is enabled, led_multi mode is forced to MANUAL automatically.
//
// MQTT commands (prefix "sensor:"):
//
//   sensor:start                   — begin continuous sampling
//   sensor:stop                    — stop sampling, LED → 0 if linked
//   sensor:read                    — single one-shot blocking read (task-safe)
//   sensor:link:on                 — enable distance → LED brightness mapping
//   sensor:link:off                — disable mapping (LED stays at last duty)
//   sensor:pin:trig:<n>            — change trigger GPIO pin (restarts task)
//   sensor:pin:echo:<n>            — change echo GPIO pin (restarts task)
//   sensor:led:pin:<n>             — change target LED pin for brightness
//   sensor:ratio:<min_cm>:<max_cm> — distance range: min=full bright, max=off
//   sensor:ratio:invert:<0|1>      — 1 = near=dim / far=bright (reversed)
//   sensor:log:on                  — start publishing distance over MQTT
//   sensor:log:off                 — stop publishing
//   sensor:log:interval:<ms>       — publish throttle (default 500 ms, min 50)
//   sensor:threshold:<cm>          — only log/alert when distance ≤ this (0=off)
//   sensor:sample:<ms>             — sampling period (default 100 ms, min 50)
//   sensor:status                  — log full config + last reading
//   sensor:health                  — publish read/timeout/noise counters
//   sensor:resetstats              — reset health counters before a retest
//
// MQTT publishes:
//   CFG_ULTRASOUND_TOPIC_DIST    — "12.4"  (cm, 1 decimal)
//   CFG_ULTRASOUND_TOPIC_STATUS  — "running dist=12.4 duty=220 linked=1"
// ─────────────────────────────────────────────────────────────────────────────

void ultrasound_init();
void ultrasound_handle();            // call every loop() iteration
void ultrasound_register_commands(); // registers "sensor:" prefix

bool ultrasound_is_running();
float ultrasound_get_distance_cm();
uint8_t ultrasound_get_duty(); // last computed LED duty (0-255)
