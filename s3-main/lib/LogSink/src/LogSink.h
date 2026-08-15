#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

struct LogEntry {
    char message[192];
};

class LogSink {
public:
    LogSink();
    void begin(size_t queueSize = 25);
    bool push(const char* formattedMsg);
    bool pop(LogEntry& entry);

private:
    QueueHandle_t m_queue;
    bool m_initialized;
};

extern LogSink g_logSink;