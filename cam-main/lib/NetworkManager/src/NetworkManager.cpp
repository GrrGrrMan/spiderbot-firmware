#include "NetworkManager.h"
#include "net_config.h"
#include "logger.h"

NetworkManager::NetworkManager()
    : m_connected(false),
      m_isHotspot(false),
      m_mqttBroker(nullptr) {}

void NetworkManager::begin() {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);

    // Register native ESP32 Wi-Fi hardware event listener
    WiFi.onEvent(NetworkManager::onWiFiEvent);

    LOG_NET("Initializing NetworkManager with WiFiMulti...");
    for (size_t i = 0; i < KNOWN_NETWORKS_COUNT; i++) {
        m_wifiMulti.addAP(KNOWN_NETWORKS[i].ssid, KNOWN_NETWORKS[i].pass);
        LOG_NET("Added AP candidate: '%s'", KNOWN_NETWORKS[i].ssid);
    }

    // Trigger initial background connection cycle
    m_wifiMulti.run();
}

void NetworkManager::update() {
    if (WiFi.status() == WL_CONNECTED) {
        if (!m_connected) {
            m_connected = true;
            updateBrokerForSSID(WiFi.SSID());
            LOG_NET("Wi-Fi Connected to '%s'! IP: %s | Broker: %s", 
                    WiFi.SSID().c_str(), 
                    WiFi.localIP().toString().c_str(), 
                    m_mqttBroker ? m_mqttBroker : "none");
        }
    } else {
        if (m_connected) {
            m_connected = false;
            m_mqttBroker = nullptr;
            LOG_ERR("Wi-Fi Connection Lost! WiFiMulti background retrying...");
        }
        // WiFiMulti handles background scanning, prioritization, and failover
        m_wifiMulti.run();
    }
}

void NetworkManager::updateBrokerForSSID(const String& ssid) {
    m_isHotspot = false;
    m_mqttBroker = nullptr;

    for (size_t i = 0; i < KNOWN_NETWORKS_COUNT; i++) {
        if (ssid.equalsIgnoreCase(KNOWN_NETWORKS[i].ssid)) {
            m_isHotspot = KNOWN_NETWORKS[i].isHotspot;
            m_mqttBroker = KNOWN_NETWORKS[i].mqttBroker;
            break;
        }
    }
}

void NetworkManager::onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            LOG_NET("[HW Event] Station GOT_IP");
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            LOG_NET("[HW Event] Station DISCONNECTED");
            break;
        default:
            break;
    }
}

bool NetworkManager::isConnected() const {
    return m_connected && (WiFi.status() == WL_CONNECTED);
}

bool NetworkManager::isHotspot() const {
    return m_isHotspot;
}

String NetworkManager::getLocalIP() const {
    return WiFi.localIP().toString();
}

const char* NetworkManager::getMQTTBroker() const {
    return m_mqttBroker;
}