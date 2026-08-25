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

    // Non-blocking queue push: Core 1 motion task never blocks on UART
    if (!g_logSink.push(fullMsg)) {
        // Fallback for early boot before task scheduler runs
        Serial.println(fullMsg);
    }
}