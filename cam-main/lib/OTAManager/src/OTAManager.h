#pragma once

#include <Arduino.h>
#include "ota_config.h"

class OTAManager {
public:
    OTAManager();
    void begin();
    void checkForUpdates(
        bool forcePrimary = false, 
        bool forceFallback = false,
        const String& customOwner = "",
        const String& customRepo = "",
        const String& customBranch = "",
        const String& customPath = "",
        const String& customPat = ""
    );
    void validateBootImage();
    bool isRunning() const;

private:
    struct TaskParams {
        bool forcePrimary;
        bool forceFallback;
        String customOwner;
        String customRepo;
        String customBranch;
        String customPath;
        String customPat;
    };

    static void otaTask(void* pvParameters);
    static bool fetchAndFlash(const OtaSourceConfig& config);
    static bool ensureTlsTime();
};

extern OTAManager otaManager;