#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <map>
#include <functional>

typedef std::function<void(const JsonDocument& doc)> CommandHandler;

class CommandDispatcher {
public:
    CommandDispatcher();
    void registerHandler(const String& commandType, CommandHandler handler);
    void dispatch(const String& type, const JsonDocument& doc);

private:
    std::map<String, CommandHandler> m_handlers;
};