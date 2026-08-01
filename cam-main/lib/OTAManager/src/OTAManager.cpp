#include "OTAManager.h"
#include "logger.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <time.h>

OTAManager otaManager;
static volatile bool s_otaRunning = false;

struct TaskParams {
    bool forcePrimary;
    bool forceFallback;
};

OTAManager::OTAManager() {}

void OTAManager::begin() {
    LOG_SYS("OTA Manager initialized.");
}

bool OTAManager::isRunning() const {
    return s_otaRunning;
}

void OTAManager::validateBootImage() {
    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            esp_ota_mark_app_valid_cancel_rollback();
            LOG_SYS("New firmware validated successfully! Rollback cancelled.");
        }
    }
}

void OTAManager::checkForUpdates(bool forcePrimary, bool forceFallback) {
    if (s_otaRunning) {
        LOG_ERR("OTA check already in progress.");
        return;
    }

    TaskParams* params = new TaskParams{forcePrimary, forceFallback};
    s_otaRunning = true;
    
    xTaskCreate([](void* p) {
        TaskParams* tp = (TaskParams*)p;
        OTAManager::otaTask(tp);
        delete tp;
        s_otaRunning = false;
        vTaskDelete(NULL);
    }, "otaTask", 8192, params, 1, NULL);
}

bool OTAManager::ensureTlsTime() {
    time_t now = 0;
    time(&now);
    if (now >= 1700000000UL) return true;

    LOG_SYS("Syncing NTP time for TLS verification...");
    configTime(0, 0, "pool.ntp.org", "time.google.com");

    uint32_t startMs = millis();
    while (millis() - startMs < 10000UL) {
        time(&now);
        if (now >= 1700000000UL) return true;
        delay(250);
    }
    LOG_ERR("NTP time sync timeout.");
    return false;
}

bool OTAManager::fetchAndFlash(const OtaSourceConfig& config) {
    // Constructs: https://raw.githubusercontent.com/YourOrg/hexapod-firmware/main/cam-main/bin/firmware
    String baseUrl = String("https://raw.githubusercontent.com/") + 
                     config.owner + "/" + 
                     config.repo + "/" + 
                     config.branch + "/" + 
                     config.projectPath + 
                     config.artifactName;

    String hashUrl = baseUrl + ".md5"; // .../cam-main/bin/firmware.md5
    String binUrl  = baseUrl + ".bin"; // .../cam-main/bin/firmware.bin

    LOG_SYS("Checking OTA Source: %s (%s)", baseUrl.c_str(), config.isPrivate ? "PRIVATE" : "PUBLIC");


    WiFiClientSecure client;
    client.setInsecure(); // Skip strict CA checking for lightweight memory usage
    HTTPClient http;

    // 1. Fetch Remote MD5 Hash
    if (!http.begin(client, hashUrl)) return false;
    if (config.isPrivate && config.authToken && strlen(config.authToken) > 0) {
        http.addHeader("Authorization", String("Bearer ") + config.authToken);
    }
    http.addHeader("Accept", "application/vnd.github.raw");

    int code = http.GET();
    if (code != 200) {
        LOG_ERR("Hash fetch failed: HTTP %d", code);
        http.end();
        return false;
    }

    String remoteMD5 = http.getString();
    remoteMD5.trim();
    http.end();

    // 2. Compare Sketch MD5 Hash
    String localMD5 = ESP.getSketchMD5();
    if (localMD5.equalsIgnoreCase(remoteMD5)) {
        LOG_SYS("Firmware is up to date (MD5 Match: %s)", localMD5.c_str());
        return true; 
    }

    LOG_SYS("MD5 Mismatch! Local: %s | Remote: %s", localMD5.c_str(), remoteMD5.c_str());
    LOG_SYS("Downloading new firmware binary...");

    // 3. Stream and Flash `.bin` Payload
    if (!http.begin(client, binUrl)) return false;
    if (config.isPrivate && config.authToken && strlen(config.authToken) > 0) {
        http.addHeader("Authorization", String("Bearer ") + config.authToken);
    }
    http.addHeader("Accept", "application/vnd.github.raw");

    code = http.GET();
    if (code != 200) {
        LOG_ERR("Binary download failed: HTTP %d", code);
        http.end();
        return false;
    }

    int totalSize = http.getSize();
    if (totalSize <= 0) {
        LOG_ERR("Invalid content length.");
        http.end();
        return false;
    }

    if (!Update.begin(totalSize)) {
        LOG_ERR("Not enough flash space for OTA update.");
        http.end();
        return false;
    }

    size_t written = Update.writeStream(*http.getStreamPtr());
    http.end();

    if (written != (size_t)totalSize) {
        LOG_ERR("OTA Written size mismatch: %d / %d", written, totalSize);
        return false;
    }

    if (Update.end() && Update.isFinished()) {
        LOG_SYS("OTA Update Successful! Rebooting in 1 second...");
        delay(1000);
        ESP.restart();
        return true;
    }

    LOG_ERR("Update failed during finalization.");
    return false;
}

void OTAManager::otaTask(void* pvParameters) {
    TaskParams* params = (TaskParams*)pvParameters;

    if (WiFi.status() != WL_CONNECTED) {
        LOG_ERR("OTA aborted — Wi-Fi not connected.");
        return;
    }

    if (!ensureTlsTime()) return;

    OtaSourceConfig primary = {
        .owner        = OTA_PRIMARY_OWNER,
        .repo         = OTA_PRIMARY_REPO,
        .branch       = OTA_PRIMARY_BRANCH,
        .projectPath  = OTA_PRIMARY_PROJECT_PATH,
        .artifactName = OTA_PRIMARY_ARTIFACT,
        .isPrivate    = OTA_PRIMARY_PRIVATE,
        .authToken    = OTA_PRIMARY_TOKEN
    };

    OtaSourceConfig fallback = {
        .owner        = OTA_FALLBACK_OWNER,
        .repo         = OTA_FALLBACK_REPO,
        .branch       = OTA_FALLBACK_BRANCH,
        .projectPath  = OTA_FALLBACK_PROJECT_PATH,
        .artifactName = OTA_FALLBACK_ARTIFACT,
        .isPrivate    = OTA_FALLBACK_PRIVATE,
        .authToken    = OTA_FALLBACK_TOKEN
    };

    // Auto-Failover Logic
    if (params->forceFallback) {
        fetchAndFlash(fallback);
    } else {
        bool success = fetchAndFlash(primary);
        if (!success && !params->forcePrimary) {
            LOG_SYS("Primary OTA failed. Triggering automatic failover to Fallback Repo...");
            fetchAndFlash(fallback);
        }
    }
}