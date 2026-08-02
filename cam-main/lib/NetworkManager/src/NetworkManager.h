#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>

class NetworkManager {
public:
    NetworkManager();
    
    void begin();
    void update();

    bool isConnected() const;
    bool isHotspot() const;
    String getLocalIP() const;
    const char* getMQTTBroker() const;

private:
    WiFiMulti m_wifiMulti;
    bool m_connected;
    bool m_isHotspot;
    const char* m_mqttBroker;
    
    void updateBrokerForSSID(const String& ssid);
    static void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info);
};

extern NetworkManager netManager;