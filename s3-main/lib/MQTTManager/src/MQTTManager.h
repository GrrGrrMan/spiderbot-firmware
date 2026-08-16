#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "net_config.h"

typedef void (*CommandCallback)(const String& type, JsonDocument& doc);
typedef void (*AudioCommandCallback)(const String& action, JsonDocument& doc);

class MQTTManager {
public:
    MQTTManager();
    
    void begin(const char* deviceId = DEVICE_ID, uint16_t brokerPort = 1883);
    void update(bool isNetworkConnected, const char* brokerHost);
    bool sendConfig();
    bool sendTelemetry(const JsonDocument& doc);
    bool sendLog(const char* logMsg);
    bool sendAudioStatus(const char* state, const char* action);
    void setCommandCallback(CommandCallback cb);
    void setAudioCommandCallback(AudioCommandCallback cb);
    bool isConnected();

private:
    WiFiClient m_netClient;
    PubSubClient m_mqttClient;
    String m_configTopic;
    String m_deviceId;
    String m_cmdTopicGlobal;
    String m_cmdTopicDevice;
    String m_telemetryTopic;
    String m_logTopic;
    String m_audioTopic;
    String m_audioStatusTopic;
    String m_currentBrokerHost;
    uint16_t m_brokerPort;
    unsigned long m_lastRetryMs;
    unsigned long m_lastTelemetryMs;
    bool m_isPublishingLog;
    CommandCallback m_cmdCallback;
    AudioCommandCallback m_audioCallback;

    void reconnect(const char* brokerHost);
    static void onMqttMessage(char* topic, byte* payload, unsigned int length);
};