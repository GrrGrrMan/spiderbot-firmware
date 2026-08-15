#include "CommandDispatcher.h"
#include "logger.h"

CommandDispatcher::CommandDispatcher() {}

void CommandDispatcher::registerHandler(const String& commandType, CommandHandler handler) {
    m_handlers[commandType] = handler;
    LOG_SYS("Registered command handler for type: '%s'", commandType.c_str());
}

void CommandDispatcher::dispatch(const String& type, const JsonDocument& doc) {
    auto it = m_handlers.find(type);
    if (it != m_handlers.end()) {
        it->second(doc);
    } else {
        LOG_ERR("No handler registered for command type: '%s'", type.c_str());
    }
}