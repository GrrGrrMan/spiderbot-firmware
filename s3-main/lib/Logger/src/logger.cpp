#include "logger.h"
#include "LogSink.h"
#include <stdarg.h>

void logPrintf(const char* tag, const char* fmt, ...) {
    if (!g_logEnabled) return;

    char buffer[192];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    char fullMsg[210];
    snprintf(fullMsg, sizeof(fullMsg), "[%s] %s", tag, buffer);

    // Push to queue for MQTT, but also print to Serial directly during early boot
    if (!g_logSink.push(fullMsg)) {
        Serial.println(fullMsg);
    } else {
        // Optional: mirror to Serial so logs are visible immediately
        Serial.println(fullMsg);
    }
}