#pragma once

#include <Arduino.h>
#include "ota_config.h"

class OTAManager {
public:
    OTAManager();
    void begin();
    void checkForUpdates(bool forcePrimary = false, bool forceFallback = false);
    void validateBootImage(); // Call after Wi-Fi/MQTT success to cancel hardware rollback
    bool isRunning() const;

private:
    static void otaTask(void* pvParameters);
    static bool fetchAndFlash(const OtaSourceConfig& config);
    static bool ensureTlsTime();
};

extern OTAManager otaManager;