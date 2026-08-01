#include "LogSink.h"

LogSink g_logSink;

LogSink::LogSink() : m_queue(NULL), m_initialized(false) {}

void LogSink::begin(size_t queueSize) {
    if (!m_initialized) {
        m_queue = xQueueCreate(queueSize, sizeof(LogEntry));
        m_initialized = (m_queue != NULL);
    }
}

bool LogSink::push(const char* formattedMsg) {
    if (!m_initialized || !m_queue) return false;

    LogEntry entry;
    snprintf(entry.message, sizeof(entry.message), "%s", formattedMsg);
    
    // Ticks = 0 ensures Core 1 motion task NEVER blocks on logging
    BaseType_t result = xQueueSend(m_queue, &entry, 0);
    return (result == pdPASS);
}

bool LogSink::pop(LogEntry& entry) {
    if (!m_initialized || !m_queue) return false;
    return (xQueueReceive(m_queue, &entry, 0) == pdPASS);
}