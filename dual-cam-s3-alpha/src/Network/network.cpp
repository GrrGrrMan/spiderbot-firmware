#include "network.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <esp_wpa2.h>
#include "Build/config/target_config.h"
#include "Build/config/secrets.h"
#include "Build/Log/logger.h"
#include <cstring>

// Credential arrays are defined in secrets.h (gitignored).
// Format: { {"SSID", "password"}, ... }
static const char *networks[][2] = WIFI_NETWORKS;

#ifndef CFG_WIFI_SCAN_WHEN_IDLE_MS
#define CFG_WIFI_SCAN_WHEN_IDLE_MS 10000UL
#endif

#ifndef CFG_WIFI_ATTEMPT_SETTLE_MS
#define CFG_WIFI_ATTEMPT_SETTLE_MS 500UL
#endif

#ifndef CFG_WIFI_RADIO_RESET_MS
#define CFG_WIFI_RADIO_RESET_MS 750UL
#endif

#ifndef CFG_WIFI_HOTSPOT_SSID
#define CFG_WIFI_HOTSPOT_SSID "spiderlink"
#endif

#ifndef CFG_WIFI_HOTSPOT_PREFER_SCAN_MS
#define CFG_WIFI_HOTSPOT_PREFER_SCAN_MS 60000UL
#endif

#ifndef CFG_WIFI_HOTSPOT_FAILED_BACKOFF_MS
#define CFG_WIFI_HOTSPOT_FAILED_BACKOFF_MS 300000UL
#endif

#ifndef CFG_WIFI_ENABLE_ENTERPRISE
#define CFG_WIFI_ENABLE_ENTERPRISE 0
#endif

struct EnterpriseNetwork
{
    const char *ssid;
    wpa2_auth_method_t method;
    const char *identity;
    const char *username;
    const char *password;
    const char *caPem;
    const char *clientCert;
    const char *clientKey;
};

#ifdef WIFI_ENTERPRISE_NETWORKS
static const EnterpriseNetwork enterpriseNetworks[] = WIFI_ENTERPRISE_NETWORKS;
#endif

static bool otaInProgress = false;
static bool otaReady = false;
static unsigned long lastWiFiRetryMs = 0;
static unsigned long lastScanStandbyLogMs = 0;
static unsigned long lastHotspotPreferScanMs = 0;
static unsigned long lastHotspotFailedMs = 0;
static volatile uint8_t lastDisconnectReason = 0;
static volatile bool disconnectSeen = false;
static bool wifiEventRegistered = false;
static bool connectedToHotspot = false;

static bool hasText(const char *value)
{
    return value && value[0] != '\0';
}

static void onWiFiDisconnected(arduino_event_id_t, arduino_event_info_t info)
{
    lastDisconnectReason = info.wifi_sta_disconnected.reason;
    disconnectSeen = true;
}

static uint8_t takeDisconnectReason()
{
    if (!disconnectSeen)
        return 0;

    disconnectSeen = false;
    return lastDisconnectReason;
}

static const char *wifiStatusName(wl_status_t status)
{
    switch (status)
    {
    case WL_IDLE_STATUS:
        return "IDLE";
    case WL_NO_SSID_AVAIL:
        return "NO_SSID";
    case WL_SCAN_COMPLETED:
        return "SCAN_DONE";
    case WL_CONNECTED:
        return "CONNECTED";
    case WL_CONNECT_FAILED:
        return "CONNECT_FAILED";
    case WL_CONNECTION_LOST:
        return "CONNECTION_LOST";
    case WL_DISCONNECTED:
        return "DISCONNECTED";
    case WL_NO_SHIELD:
        return "NO_SHIELD";
    default:
        return "UNKNOWN";
    }
}

static bool waitForConnection()
{
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < CFG_WIFI_CONNECT_ATTEMPTS)
    {
        delay(500);
        attempts++;
    }
    return WiFi.status() == WL_CONNECTED;
}

static bool isHotspotSsid(const char *ssid)
{
    return hasText(ssid) && strcmp(ssid, CFG_WIFI_HOTSPOT_SSID) == 0;
}

static bool scanHasSsid(const char *ssid, int scanCount)
{
    if (!hasText(ssid) || scanCount <= 0)
        return false;

    for (int i = 0; i < scanCount; i++)
    {
        if (WiFi.SSID(i) == ssid)
            return true;
    }
    return false;
}

static void stopStaAttempt();
static void logConnectFailure(const char *ssid, bool enterprise);

static int scanForKnownNetworks()
{
    LOG("[WiFi] scanning for whitelisted networks");
    return WiFi.scanNetworks(false, true);
}

static bool tryPersonalNetwork(const char *ssid, const char *password, int scanCount)
{
    if (!scanHasSsid(ssid, scanCount))
        return false;

    LOGF("[WiFi] Trying %s...\n", ssid);
    esp_wifi_sta_wpa2_ent_disable();
    disconnectSeen = false;
    WiFi.begin(ssid, password);

    if (waitForConnection())
    {
        LOGF("[WiFi] Connected to %s, IP: %s\n",
             ssid, WiFi.localIP().toString().c_str());
        connectedToHotspot = isHotspotSsid(ssid);
        if (connectedToHotspot)
            lastHotspotFailedMs = 0;
        lastWiFiRetryMs = millis();
        return true;
    }

    logConnectFailure(ssid, false);
    if (isHotspotSsid(ssid))
        lastHotspotFailedMs = millis();
    stopStaAttempt();
    return false;
}

static void prepareStaRadio()
{
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(false);
    WiFi.setSleep(false);
}

static void stopStaAttempt()
{
    WiFi.disconnect(false, false);
    esp_wifi_sta_wpa2_ent_disable();
    delay(CFG_WIFI_ATTEMPT_SETTLE_MS);
}

static void pauseStaAfterFailure()
{
    WiFi.disconnect(true, false);
    esp_wifi_sta_wpa2_ent_disable();
    delay(CFG_WIFI_RADIO_RESET_MS);
}

static void logConnectFailure(const char *ssid, bool enterprise)
{
    const uint8_t reason = takeDisconnectReason();
    wl_status_t status = WiFi.status();
    if (reason)
    {
        LOGF("[WiFi] Failed to connect to %s%s after %d attempts (status=%s, reason=%u/%s), trying next...\n",
             enterprise ? "enterprise " : "",
             ssid,
             CFG_WIFI_CONNECT_ATTEMPTS,
             wifiStatusName(status),
             reason,
             WiFi.disconnectReasonName((wifi_err_reason_t)reason));
    }
    else
    {
        LOGF("[WiFi] Failed to connect to %s%s after %d attempts (status=%s), trying next...\n",
             enterprise ? "enterprise " : "",
             ssid,
             CFG_WIFI_CONNECT_ATTEMPTS,
             wifiStatusName(status));
    }
}

static bool connectWiFi()
{
    if (WiFi.status() == WL_CONNECTED)
        return true;

    prepareStaRadio();
    connectedToHotspot = false;

    const int scanCount = scanForKnownNetworks();
    bool sawKnownNetwork = false;

    for (auto &net : networks)
    {
        if (!isHotspotSsid(net[0]) || !scanHasSsid(net[0], scanCount))
            continue;

        sawKnownNetwork = true;
        if (tryPersonalNetwork(net[0], net[1], scanCount))
        {
            WiFi.scanDelete();
            return true;
        }
    }

    for (auto &net : networks)
    {
        if (isHotspotSsid(net[0]) || !scanHasSsid(net[0], scanCount))
            continue;

        sawKnownNetwork = true;
        if (tryPersonalNetwork(net[0], net[1], scanCount))
        {
            WiFi.scanDelete();
            return true;
        }
    }

#if CFG_WIFI_ENABLE_ENTERPRISE && defined(WIFI_ENTERPRISE_NETWORKS)
    for (const auto &net : enterpriseNetworks)
    {
        if (!hasText(net.ssid) || !hasText(net.username) || !hasText(net.password))
        {
            LOG("[WiFi] Skipping enterprise network with missing SSID, username, or password");
            continue;
        }
        if (!scanHasSsid(net.ssid, scanCount))
            continue;

        sawKnownNetwork = true;
        const char *identity = hasText(net.identity) ? net.identity : "";
        const char *clientCert = (hasText(net.clientCert) && hasText(net.clientKey))
                                     ? net.clientCert
                                     : nullptr;
        const char *clientKey = clientCert ? net.clientKey : nullptr;

        LOGF("[WiFi] Trying enterprise %s...\n", net.ssid);
        disconnectSeen = false;
        WiFi.begin(net.ssid,
                   net.method,
                   identity,
                   net.username,
                   net.password,
                   net.caPem,
                   clientCert,
                   clientKey);

        if (waitForConnection())
        {
            LOGF("[WiFi] Connected to enterprise %s, IP: %s\n",
                 net.ssid, WiFi.localIP().toString().c_str());
            connectedToHotspot = isHotspotSsid(net.ssid);
            lastWiFiRetryMs = millis();
            WiFi.scanDelete();
            return true;
        }

        logConnectFailure(net.ssid, true);
        stopStaAttempt();
    }
#endif

    if (!sawKnownNetwork)
    {
        const unsigned long now = millis();
        if (lastScanStandbyLogMs == 0 ||
            (now - lastScanStandbyLogMs) >= CFG_WIFI_SCAN_WHEN_IDLE_MS)
        {
            lastScanStandbyLogMs = now;
            LOG("[WiFi] No whitelisted networks visible - scan standby");
        }
    }
    else
    {
        LOG("All visible whitelisted networks failed");
    }

    WiFi.scanDelete();
    pauseStaAfterFailure();
    lastWiFiRetryMs = millis();
    return false;
}

static void preferHotspotWhenVisible()
{
    if (connectedToHotspot || !hasText(CFG_WIFI_HOTSPOT_SSID) ||
        WiFi.status() != WL_CONNECTED)
    {
        return;
    }

    const unsigned long now = millis();
    if (lastHotspotFailedMs != 0 &&
        (now - lastHotspotFailedMs) < CFG_WIFI_HOTSPOT_FAILED_BACKOFF_MS)
    {
        return;
    }
    if (lastHotspotPreferScanMs != 0 &&
        (now - lastHotspotPreferScanMs) < CFG_WIFI_HOTSPOT_PREFER_SCAN_MS)
    {
        return;
    }
    lastHotspotPreferScanMs = now;

    const int scanCount = WiFi.scanNetworks(false, true);
    const bool hotspotVisible = scanHasSsid(CFG_WIFI_HOTSPOT_SSID, scanCount);
    WiFi.scanDelete();
    if (!hotspotVisible)
        return;

    LOG("[WiFi] Preferred hotspot visible - switching to hotspot");
    WiFi.disconnect(false, false);
    connectedToHotspot = false;
    lastWiFiRetryMs = 0;
    delay(CFG_WIFI_ATTEMPT_SETTLE_MS);
    connectWiFi();
}

static void setupOTA()
{
    if (otaReady)
        return;

    if (WiFi.status() != WL_CONNECTED)
    {
        LOG("OTA deferred - WiFi disconnected");
        return;
    }

    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA.onStart([]()
    {
        otaInProgress = true;
        LOG("OTA starting...");
    });
    ArduinoOTA.onEnd([]()
    {
        otaInProgress = false;
        LOG("OTA done, rebooting...");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
    {
        LOGF("OTA: %u%%\n", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error)
    {
        otaInProgress = false;
        LOGF("OTA Error: %u\n", error);
    });

    ArduinoOTA.begin();
    otaReady = true;
    LOG("OTA ready");
}

void network_init()
{
    WiFi.mode(WIFI_STA);
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    WiFi.setSleep(false);
    if (!wifiEventRegistered)
    {
        WiFi.onEvent(onWiFiDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
        wifiEventRegistered = true;
    }

    if (!connectWiFi())
        LOGF("[WiFi] Will retry every %lus\n", CFG_WIFI_RETRY_MS / 1000UL);

    setupOTA();
}

void network_handle()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        connectedToHotspot = false;
        unsigned long now = millis();
        if (lastWiFiRetryMs == 0 || (now - lastWiFiRetryMs) >= CFG_WIFI_RETRY_MS)
        {
            LOG("[WiFi] disconnected - scan retry");
            connectWiFi();
        }
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        preferHotspotWhenVisible();
        setupOTA();
        ArduinoOTA.handle();
    }
}


bool network_ota_in_progress()
{
    return otaInProgress;
}

bool network_connected()
{
    return WiFi.status() == WL_CONNECTED;
}

bool network_is_hotspot()
{
    return network_connected() && connectedToHotspot;
}
