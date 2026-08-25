#include "MQTTManager.h"
#include "logger.h"
#include "cmd_schema.h"
#include "NetworkManager.h"
#include <ESPmDNS.h> 

static MQTTManager* s_instance = nullptr;

MQTTManager::MQTTManager()
    : m_mqttClient(m_netClient),
      m_deviceId(DEVICE_ID),
      m_currentBrokerHost(""),
      m_brokerPort(1883),
      m_lastRetryMs(0),
      m_lastTelemetryMs(0),
      m_isPublishingLog(false),
      m_cmdCallback(nullptr),
      m_audioCallback(nullptr),
      m_audioBinCallback(nullptr) {
    s_instance = this;
}

void MQTTManager::begin(const char* deviceId, uint16_t brokerPort) {
    m_deviceId = String(deviceId);
    m_brokerPort = brokerPort;

    m_configTopic    = "hexapod/" + m_deviceId + "/config";
    m_cmdTopicGlobal = "hexapod/cmd";
    m_cmdTopicDevice = "hexapod/" + m_deviceId + "/cmd";
    m_telemetryTopic = "hexapod/" + m_deviceId + "/telemetry";
    m_logTopic       = "hexapod/" + m_deviceId + "/logs";
    m_audioTopic     = "hexapod/" + m_deviceId + "/audio";
    m_audioStatusTopic = "hexapod/" + m_deviceId + "/audio/status";

    MDNS.begin("hexapod-s3-client"); // Initialized once here

    m_mqttClient.setCallback(MQTTManager::onMqttMessage);
    // P5: audio topic carries chunked TTS frames (~4 KB base64 payload).
    // 8 KB comfortably holds one frame + JSON envelope (PubSubClient caps at 64 KB).
    m_mqttClient.setBufferSize(8192);
}

void MQTTManager::setCommandCallback(CommandCallback cb) {
    m_cmdCallback = cb;
}

void MQTTManager::setAudioCommandCallback(AudioCommandCallback cb) {
    m_audioCallback = cb;
}

void MQTTManager::setAudioBinCommandCallback(AudioBinCommandCallback cb) {
    m_audioBinCallback = cb;
}

void MQTTManager::reconnect(const char* brokerHost) {
    if (m_mqttClient.connected()) return;

    // Generate unique Client ID using ESP32 MAC address to prevent public broker disconnect collisions
    String uniqueClientId = m_deviceId + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    bool connected = false;

    // Iterate through configured candidates sequentially
    for (uint8_t i = 0; i < MAX_BROKERS_PER_SSID; i++) {
        const char* brokerCandidate = netManager.getMQTTBroker(i);
        if (!brokerCandidate) break; // End of configured candidates list

        String hostStr = String(brokerCandidate);
        IPAddress targetIP;

        // 1. mDNS filter: Check if the .local host is active (150ms timeout)
        if (hostStr.endsWith(".local") || hostStr.indexOf('.') == -1) {
            String hostname = hostStr;
            if (hostname.endsWith(".local")) {
                hostname = hostname.substring(0, hostname.length() - 6); // Strip ".local"
            }

            // Perform non-blocking mDNS query
            targetIP = MDNS.queryHost(hostname, 150); 
            
            if (targetIP.toString() == "0.0.0.0") {
                LOG_NET("Broker %s is offline (mDNS failed in 150ms). Skipping...", brokerCandidate);
                continue; // Skip offline candidate instantly
            }
            
            LOG_NET("Broker %s is ONLINE at IP: %s.", brokerCandidate, targetIP.toString().c_str());
            m_mqttClient.setServer(targetIP, m_brokerPort);
            m_currentBrokerHost = targetIP.toString();
        } else {
            // For raw IPs or external cloud brokers
            m_mqttClient.setServer(brokerCandidate, m_brokerPort);
            m_currentBrokerHost = hostStr;
        }

        LOG_NET("Connecting to MQTT Broker at %s:%d as [%s]...", brokerCandidate, m_brokerPort, uniqueClientId.c_str());

        // 2. Connect handshake: Only connect to verified online IPs
        if (m_mqttClient.connect(uniqueClientId.c_str())) {
            LOG_NET("MQTT Connected successfully to: %s!", brokerCandidate);
            m_mqttClient.subscribe(m_cmdTopicGlobal.c_str());
            m_mqttClient.subscribe(m_cmdTopicDevice.c_str());
            m_mqttClient.subscribe(m_audioTopic.c_str());
            LOG_NET("Subscribed to [%s] & [%s] & [%s]", m_cmdTopicGlobal.c_str(), m_cmdTopicDevice.c_str(), m_audioTopic.c_str());
            connected = true;
            break; // Exit candidate loop on successful connection
        } else {
            LOG_ERR("MQTT connect failed for %s, state=%d", brokerCandidate, m_mqttClient.state());
        }
    }

    if (!connected) {
        LOG_ERR("No active broker candidates could be reached on this network.");
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

bool MQTTManager::sendAudioStatus(const char* state, const char* action) {
    if (!m_mqttClient.connected()) return false;

    JsonDocument doc;
    doc["state"] = state;
    if (action) doc["action"] = action;

    char buffer[128];
    size_t bytesWritten = serializeJson(doc, buffer, sizeof(buffer));
    if (bytesWritten == 0) return false;

    return m_mqttClient.publish(m_audioStatusTopic.c_str(), buffer);
}

bool MQTTManager::sendTelemetry(const JsonDocument& doc) {
    if (!m_mqttClient.connected()) return false;

    unsigned long now = millis();
    if (now - m_lastTelemetryMs < 100) {
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

    if (strcmp(topic, s_instance->m_audioTopic.c_str()) == 0) {
        // QUICK-ROUTE: If magic byte 0xAA is present, handle natively
        if (length > 0 && payload[0] == 0xAA) {
            if (s_instance->m_audioBinCallback) {
                s_instance->m_audioBinCallback((const uint8_t*)payload, length);
            }
            return;
        }

        // FALLBACK: Web UI still sends {"action": "beep"} as JSON
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, payload, length);
        if (!err && s_instance->m_audioCallback) {
            const char* action = doc["action"] | "unknown";
            LOG_NET("MQTT Received [%s] -> Action: '%s'", topic, action);
            s_instance->m_audioCallback(String(action), doc);
        } else if (err) {
            LOG_ERR("JSON parse failed on topic [%s]: %s", topic, err.c_str());
        }
        return;
    }

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