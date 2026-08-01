#include "NetworkManager.h"
#include "net_config.h"
#include "logger.h"
#include <WiFi.h>

NetworkManager::NetworkManager() 
    : m_state(NetState::IDLE),
      m_lastRetryMs(0),
      m_targetNetIndex(-1),
      m_connectAttempts(0),
      m_isHotspot(false),
      m_mqttBroker("pi-hub.local") {}

void NetworkManager::begin() {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(false);
    WiFi.setSleep(false);
    
    LOG_NET("Initializing Network Manager...");
    startAsyncScan();
}

void NetworkManager::startAsyncScan() {
    LOG_NET("Starting async network scan...");
    WiFi.scanNetworks(true);
    m_state = NetState::SCANNING;
}

void NetworkManager::processScanResults() {
    int found = WiFi.scanComplete();
    if (found == WIFI_SCAN_RUNNING) return;

    if (found < 0) {
        LOG_NET("Scan failed or returned 0 networks.");
        WiFi.scanDelete();
        m_state = NetState::IDLE;
        return;
    }

    m_targetNetIndex = -1;

    for (size_t i = 0; i < KNOWN_NETWORKS_COUNT; i++) {
        for (int j = 0; j < found; j++) {
            if (WiFi.SSID(j) == KNOWN_NETWORKS[i].ssid) {
                m_targetNetIndex = (int)i;
                break;
            }
        }
        if (m_targetNetIndex != -1) break;
    }

    WiFi.scanDelete();

    if (m_targetNetIndex != -1) {
        LOG_NET("Match found: '%s'! Connecting...", KNOWN_NETWORKS[m_targetNetIndex].ssid);
        WiFi.begin(KNOWN_NETWORKS[m_targetNetIndex].ssid, KNOWN_NETWORKS[m_targetNetIndex].pass);
        m_connectAttempts = 0;
        m_state = NetState::CONNECTING;
    } else {
        LOG_NET("No known networks found in range.");
        m_state = NetState::IDLE;
    }
}

void NetworkManager::update() {
    switch (m_state) {
        case NetState::IDLE: {
            if (WiFi.status() == WL_CONNECTED) {
                m_state = NetState::CONNECTED;
            } else if (millis() - m_lastRetryMs >= 10000UL) {
                m_lastRetryMs = millis();
                startAsyncScan();
            }
            break;
        }

        case NetState::SCANNING: {
            processScanResults();
            break;
        }

        case NetState::CONNECTING: {
            if (WiFi.status() == WL_CONNECTED) {
                m_state = NetState::CONNECTED;
                m_isHotspot = KNOWN_NETWORKS[m_targetNetIndex].isHotspot;
                m_mqttBroker = KNOWN_NETWORKS[m_targetNetIndex].mqttBroker;
                LOG_NET("Connected to %s! IP: %s", 
                        KNOWN_NETWORKS[m_targetNetIndex].ssid, 
                        WiFi.localIP().toString().c_str());
            } else {
                m_connectAttempts++;
                if (m_connectAttempts >= 30) {
                    LOG_ERR("Failed to connect to %s", KNOWN_NETWORKS[m_targetNetIndex].ssid);
                    m_state = NetState::IDLE;
                    m_lastRetryMs = millis();
                }
            }
            break;
        }

        case NetState::CONNECTED: {
            if (WiFi.status() != WL_CONNECTED) {
                LOG_ERR("Wi-Fi connection lost!");
                m_state = NetState::IDLE;
                m_lastRetryMs = millis();
            }
            break;
        }
    }
}

bool NetworkManager::isConnected() const {
    return (m_state == NetState::CONNECTED) && (WiFi.status() == WL_CONNECTED);
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