#include "MQTTManager.h"
#include "logger.h"
#include "cmd_schema.h"

static MQTTManager* s_instance = nullptr;

MQTTManager::MQTTManager()
    : m_mqttClient(m_netClient),
      m_deviceId(DEVICE_ID),
      m_currentBrokerHost(""),
      m_brokerPort(1883),
      m_lastRetryMs(0),
      m_lastTelemetryMs(0),
      m_isPublishingLog(false),
      m_cmdCallback(nullptr) {
    s_instance = this;
}

void MQTTManager::begin(const char* deviceId, uint16_t brokerPort) {
    m_configTopic = "hexapod/" + m_deviceId + "/config";

    m_deviceId = String(deviceId);
    m_brokerPort = brokerPort;

    m_cmdTopicGlobal = "hexapod/cmd";
    m_cmdTopicDevice = "hexapod/" + m_deviceId + "/cmd";
    m_telemetryTopic = "hexapod/" + m_deviceId + "/telemetry";
    m_logTopic       = "hexapod/" + m_deviceId + "/logs";

    m_mqttClient.setCallback(MQTTManager::onMqttMessage);
    m_mqttClient.setBufferSize(1024);
}

void MQTTManager::setCommandCallback(CommandCallback cb) {
    m_cmdCallback = cb;
}

void MQTTManager::reconnect(const char* brokerHost) {
    if (m_mqttClient.connected()) return;

    if (m_currentBrokerHost != brokerHost) {
        m_currentBrokerHost = brokerHost;
        m_mqttClient.setServer(m_currentBrokerHost.c_str(), m_brokerPort);
    }

    // Generate unique Client ID using ESP32 MAC address to prevent public broker disconnect collisions
    String uniqueClientId = m_deviceId + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    LOG_NET("Connecting to MQTT Broker at %s:%d as [%s]...", m_currentBrokerHost.c_str(), m_brokerPort, uniqueClientId.c_str());

    if (m_mqttClient.connect(uniqueClientId.c_str())) {
        LOG_NET("MQTT Connected successfully!");
        m_mqttClient.subscribe(m_cmdTopicGlobal.c_str());
        m_mqttClient.subscribe(m_cmdTopicDevice.c_str());
        LOG_NET("Subscribed to [%s] & [%s]", m_cmdTopicGlobal.c_str(), m_cmdTopicDevice.c_str());
    } else {
        LOG_ERR("MQTT connect failed, state=%d", m_mqttClient.state());
    }
}

void MQTTManager::update(bool isNetworkConnected, const char* brokerHost) {
    if (!isNetworkConnected || !brokerHost || strlen(brokerHost) == 0) return;

    if (!m_mqttClient.connected()) {
        unsigned long now = millis();
        if (now - m_lastRetryMs >= 5000UL) {
            m_lastRetryMs = now;
            reconnect(brokerHost);
        }
    } else {
        m_mqttClient.loop();
    }
}

bool MQTTManager::sendLog(const char* logMsg) {
    if (!m_mqttClient.connected() || m_isPublishingLog) return false;

    m_isPublishingLog = true; // Guard prevents infinite recursion on log failure
    bool success = m_mqttClient.publish(m_logTopic.c_str(), logMsg);
    m_isPublishingLog = false;
    
    return success;
}

bool MQTTManager::sendTelemetry(const JsonDocument& doc) {
    if (!m_mqttClient.connected()) return false;

    unsigned long now = millis();
    if (now - m_lastTelemetryMs < 500) {
        return false;
    }

    JsonDocument copyDoc = doc;
    copyDoc["device_id"] = m_deviceId;

    char buffer[512];
    size_t bytesWritten = serializeJson(copyDoc, buffer, sizeof(buffer));
    if (bytesWritten == 0) return false;

    bool success = m_mqttClient.publish(m_telemetryTopic.c_str(), buffer);
    if (success) {
        m_lastTelemetryMs = now;
    }
    return success;
}

bool MQTTManager::isConnected() {
    return m_mqttClient.connected();
}

bool MQTTManager::sendConfig() {
    if (!m_mqttClient.connected()) return false;

    JsonDocument doc;
    buildConfigPayload(doc, m_deviceId);

    char buffer[512];
    size_t bytesWritten = serializeJson(doc, buffer, sizeof(buffer));
    if (bytesWritten == 0) return false;

    bool success = m_mqttClient.publish(m_configTopic.c_str(), (const uint8_t*)buffer, bytesWritten, true);
    if (success) {
        LOG_NET("Retained hardware configuration published to [%s]", m_configTopic.c_str());
    }
    return success;
}

void MQTTManager::onMqttMessage(char* topic, byte* payload, unsigned int length) {
    if (!s_instance) return;

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload, length);
    if (err) {
        LOG_ERR("JSON parse failed on topic [%s]: %s", topic, err.c_str());
        return;
    }

    const char* type = doc["type"] | "unknown";

    if (strcmp(type, "heartbeat") != 0) {
        LOG_NET("MQTT Received [%s] -> Type: '%s'", topic, type);
    }

    if (s_instance->m_cmdCallback) {
        s_instance->m_cmdCallback(String(type), doc);
    }
}
