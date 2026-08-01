#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
// log_sink.h — lightweight one-slot sink for the LOG() macro.
//
// Any module that wants to forward log lines somewhere (MQTT, SD card, BLE…)
// calls log_sink_register() once during its init.  The LOG() macro in
// logger.h calls log_sink_send() automatically — no direct dependency on
// the forwarding module is needed, so logger.h stays network-free.
//
// Only one sink is supported at a time.  Register the most important one last
// if you ever chain them (it wins).
// ─────────────────────────────────────────────────────────────────────────────

typedef void (*LogSinkFn)(const String &msg);

// Register a forwarding function.  Pass nullptr to clear.
void log_sink_register(LogSinkFn fn);

// Called by LOG() / LOGF() — forwards to the registered sink if one exists.
void log_sink_send(const String &msg);
