#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
// safeboot.h — Crash-loop recovery guard.
//
// Problem: if any driver in setup() causes a panic, the device reboots and
// immediately crashes again — loop is never reached, ArduinoOTA.handle() never
// runs, and the device is permanently stuck until physically reflashed.
//
// Solution: count consecutive panics in RTC memory (survives soft resets).
// After CFG_SAFEBOOT_THRESHOLD crashes, enter SAFE MODE:
//   • WiFi connects normally
//   • ArduinoOTA runs (push OTA via PlatformIO)
//   • MQTT connects — "reset" command still works
//   • ALL other subsystem inits are skipped
//
// Safe mode persists across reboots until a successful flash+boot clears it.
// The crash counter also auto-clears if the device runs stably for
// CFG_SAFEBOOT_STABLE_MS without panicking (see safeboot_update()).
//
// Typical flow:
//   setup()  → safeboot_check()   — returns true if safe mode, skip drivers
//   loop()   → safeboot_update()  — clears counter after stable uptime
//   setup() end (normal mode only) is NOT required to call anything.
// ─────────────────────────────────────────────────────────────────────────────

// Returns true if safe mode is active — caller must skip all driver inits.
// Must be called as early as possible in setup(), before any driver init.
bool safeboot_check();

// Call once per loop() iteration.  Clears the crash counter after the device
// has run stably for CFG_SAFEBOOT_STABLE_MS.  No-op in safe mode.
void safeboot_update();

// Returns true if currently running in safe mode (no drivers initialised).
bool safeboot_active();
