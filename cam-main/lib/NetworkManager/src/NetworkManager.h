#pragma once

#include <Arduino.h>

enum class NetState {
    IDLE,
    SCANNING,
    CONNECTING,
    CONNECTED
};

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
    NetState m_state;
    unsigned long m_lastRetryMs;
    int m_targetNetIndex;
    int m_connectAttempts;
    bool m_isHotspot;
    const char* m_mqttBroker;

    void startAsyncScan();
    void processScanResults();
};