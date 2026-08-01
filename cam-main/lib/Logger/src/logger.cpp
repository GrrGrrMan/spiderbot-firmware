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

    // 1. Output to local Serial Monitor
    Serial.printf("[%s] %s\r\n", tag, buffer);

    // 2. Push to FreeRTOS Queue for remote MQTT streaming
    char fullMsg[210];
    snprintf(fullMsg, sizeof(fullMsg), "[%s] %s", tag, buffer);
    g_logSink.push(fullMsg);
}