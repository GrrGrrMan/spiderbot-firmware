#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "net_config.h"

typedef void (*CommandCallback)(const String& type, JsonDocument& doc);

class MQTTManager {
public:
    MQTTManager();
    
    void begin(const char* deviceId = DEVICE_ID, uint16_t brokerPort = 1883);
    void update(bool isNetworkConnected, const char* brokerHost);
    bool sendTelemetry(const JsonDocument& doc);
    bool sendLog(const char* logMsg);
    void setCommandCallback(CommandCallback cb);
    bool isConnected();

private:
    WiFiClient m_netClient;
    PubSubClient m_mqttClient;
    String m_deviceId;
    String m_cmdTopicGlobal;
    String m_cmdTopicDevice;
    String m_telemetryTopic;
    String m_logTopic;
    String m_currentBrokerHost;
    uint16_t m_brokerPort;
    unsigned long m_lastRetryMs;
    unsigned long m_lastTelemetryMs;
    bool m_isPublishingLog;
    CommandCallback m_cmdCallback;

    void reconnect(const char* brokerHost);
    static void onMqttMessage(char* topic, byte* payload, unsigned int length);
};