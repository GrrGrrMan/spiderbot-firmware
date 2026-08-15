#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include "net_config.h"

class NetworkManager {
public:
    NetworkManager();
    
    void begin();
    void update();

    bool isConnected() const;
    bool isHotspot() const;
    String getLocalIP() const;
    const char* getMQTTBroker(uint8_t index = 0) const;

private:
    WiFiMulti m_wifiMulti;
    bool m_connected;
    bool m_isHotspot;
     const char* m_mqttBrokers[MAX_BROKERS_PER_SSID];
    
    void updateBrokerForSSID(const String& ssid);
    static void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info);
};

extern NetworkManager netManager;