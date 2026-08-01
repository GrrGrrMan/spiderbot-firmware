#pragma once
#include <Arduino.h>
#include "Build/Log/log_sink.h"

// ─────────────────────────────────────────────────────────────────────────────
// logger.h — drop-in replacement for Serial.println() that also forwards
// to any registered log sink (e.g. MQTT via mqtt_trigger.cpp).
//
// Usage:
//   LOG("Booting...");
//   LOGF("[OTA] Heap: %u bytes", ESP.getFreeHeap());
//
// No direct dependency on Network/ — the sink is wired up at runtime by
// whoever calls log_sink_register() (typically mqtt_trigger_init()).
// ─────────────────────────────────────────────────────────────────────────────

#define LOG(msg)                          \
    do                                    \
    {                                     \
        String _m = String(msg);          \
        Serial.println(_m);               \
        log_sink_send(_m);                \
    } while (0)

#define LOGF(fmt, ...)                                     \
    do                                                     \
    {                                                      \
        char _b[256];                                      \
        snprintf(_b, sizeof(_b), fmt, ##__VA_ARGS__);      \
        Serial.print(_b);                                  \
        log_sink_send(String(_b));                         \
    } while (0)
