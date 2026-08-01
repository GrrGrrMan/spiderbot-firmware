#include "ota_pull.h"
#include "Build/OTA/certs.h"
#include "Build/config/secrets.h"
#include "Build/Log/logger.h"
#include "Build/config/target_config.h"
#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Update.h>
#include <WiFi.h>
#include <time.h>

#ifndef CFG_OTA_TIME_SYNC_TIMEOUT_MS
#define CFG_OTA_TIME_SYNC_TIMEOUT_MS 12000UL
#endif

#ifndef CFG_OTA_TIME_MIN_EPOCH
#define CFG_OTA_TIME_MIN_EPOCH 1700000000UL
#endif

#ifndef CFG_OTA_NTP_SERVER_1
#define CFG_OTA_NTP_SERVER_1 "pool.ntp.org"
#endif

#ifndef CFG_OTA_NTP_SERVER_2
#define CFG_OTA_NTP_SERVER_2 "time.google.com"
#endif

#ifndef CFG_OTA_NTP_SERVER_3
#define CFG_OTA_NTP_SERVER_3 "time.cloudflare.com"
#endif

// ── Primary repo URLs (private — auth token required) ────────────────────────
#define OTA_HASH_URL \
    "https://raw.githubusercontent.com/" CFG_OTA_REPO_OWNER "/" CFG_OTA_REPO_NAME "/" CFG_OTA_BRANCH "/" CFG_OTA_ARTIFACT_BASENAME ".md5"

#define OTA_BIN_URL \
    "https://raw.githubusercontent.com/" CFG_OTA_REPO_OWNER "/" CFG_OTA_REPO_NAME "/" CFG_OTA_BRANCH "/" CFG_OTA_ARTIFACT_BASENAME ".bin"

// ── Fallback repo URLs (public — no auth token) ───────────────────────────────
#define OTA_FALLBACK_HASH_URL \
    "https://raw.githubusercontent.com/" CFG_OTA_FALLBACK_REPO_OWNER "/" CFG_OTA_FALLBACK_REPO_NAME "/" CFG_OTA_FALLBACK_BRANCH "/" CFG_OTA_ARTIFACT_BASENAME ".md5"

#define OTA_FALLBACK_BIN_URL \
    "https://raw.githubusercontent.com/" CFG_OTA_FALLBACK_REPO_OWNER "/" CFG_OTA_FALLBACK_REPO_NAME "/" CFG_OTA_FALLBACK_BRANCH "/" CFG_OTA_ARTIFACT_BASENAME ".bin"

// ── State ────────────────────────────────────────────────────────────────────
static volatile bool otaRunning = false;
static volatile uint32_t lastCheck = 0;
static volatile bool forceCheck = false;
static volatile bool forceFallback = false; // true = use fallback URLs, no auth
static bool lastWasUpToDate = false;
static OtaMode otaMode = OtaMode::AUTO;

bool ota_pull_in_progress() { return otaRunning; }

static bool tlsTimeReady()
{
    time_t now = 0;
    time(&now);
    return now >= (time_t)CFG_OTA_TIME_MIN_EPOCH;
}

static bool ensureTlsTime()
{
    if (tlsTimeReady())
        return true;

    LOG("[OTA] Syncing time for TLS...");
    configTime(0, 0, CFG_OTA_NTP_SERVER_1, CFG_OTA_NTP_SERVER_2, CFG_OTA_NTP_SERVER_3);

    const uint32_t startMs = millis();
    while ((uint32_t)(millis() - startMs) < CFG_OTA_TIME_SYNC_TIMEOUT_MS)
    {
        if (tlsTimeReady())
        {
            time_t now = 0;
            time(&now);
            LOGF("[OTA] TLS time ready: %lu\n", (unsigned long)now);
            return true;
        }
        delay(250);
    }

    LOG("[OTA] Time sync failed; HTTPS verification deferred");
    return false;
}

// ── Shared helper — TLS request, optionally authenticated ────────────────────
static bool beginSecureRequest(WiFiClientSecure &client,
                               HTTPClient &http,
                               const char *url,
                               int timeoutMs,
                               bool withAuth)
{
    client.setCACert(GITHUB_ROOT_CA);
    if (!http.begin(client, url))
        return false;

    if (withAuth)
        http.addHeader("Authorization", "Bearer " + String(GITHUB_TOKEN));

    http.addHeader("Accept", "application/vnd.github.raw");
    http.setTimeout(timeoutMs);
    return true;
}

// ── FreeRTOS task ─────────────────────────────────────────────────────────────
static void otaTask(void *)
{
    // Capture source at task entry — otaRunning prevents any concurrent write
    // to forceFallback while we are running, so this read is safe.
    const bool useFallback = forceFallback;
    const bool addAuth = !useFallback;
    const char *hashUrl = useFallback ? OTA_FALLBACK_HASH_URL : OTA_HASH_URL;
    const char *binUrl = useFallback ? OTA_FALLBACK_BIN_URL : OTA_BIN_URL;

    LOG(useFallback ? "[OTA] Source: FALLBACK (public repo)"
                    : "[OTA] Source: PRIMARY (private repo)");

    if (ESP.getFreeHeap() < CFG_OTA_MIN_HEAP || WiFi.status() != WL_CONNECTED)
    {
        LOG("[OTA] Aborted — low heap or WiFi disconnected");
        otaRunning = false;
        vTaskDelete(nullptr);
        return;
    }

    if (!ensureTlsTime())
    {
        otaRunning = false;
        vTaskDelete(nullptr);
        return;
    }

    // 1. Fetch remote MD5 ───────────────────────────────────────────────────
    WiFiClientSecure hashClient;
    HTTPClient hashHttp;

    if (!beginSecureRequest(hashClient, hashHttp, hashUrl,
                            CFG_OTA_HASH_TIMEOUT, addAuth))
    {
        LOG("[OTA] Failed to begin hash request");
        otaRunning = false;
        vTaskDelete(nullptr);
        return;
    }

    int code = hashHttp.GET();
    if (code != 200)
    {
        LOGF("[OTA] Hash fetch failed: HTTP %d\n", code);
        if (code == 404 && !useFallback)
            LOG("[OTA] Tip: firmware.md5 missing — run  pio run  to build and push it");
        if (code == 401 || code == 404)
            LOG("[OTA] Tip: if token was rotated, send  ota:fallback  to use public repo");
        hashHttp.end();
        otaRunning = false;
        vTaskDelete(nullptr);
        return;
    }

    String remoteMD5 = hashHttp.getString();
    remoteMD5.trim();
    hashHttp.end();

    // 2. Compare against running sketch ────────────────────────────────────
    String localMD5 = ESP.getSketchMD5();

    if (localMD5 == remoteMD5)
    {
        if (!lastWasUpToDate)
        {
            LOG("[OTA] Up to date");
            lastWasUpToDate = true;
        }
        otaRunning = false;
        vTaskDelete(nullptr);
        return;
    }

    lastWasUpToDate = false;
    LOG("[OTA] Local:  " + localMD5);
    LOG("[OTA] Remote: " + remoteMD5);
    LOG("[OTA] New firmware detected — downloading...");

    // 3. Download and flash ────────────────────────────────────────────────
    WiFiClientSecure binClient;
    HTTPClient binHttp;

    if (!beginSecureRequest(binClient, binHttp, binUrl,
                            CFG_OTA_DL_TIMEOUT, addAuth))
    {
        LOG("[OTA] Failed to begin download request");
        otaRunning = false;
        vTaskDelete(nullptr);
        return;
    }

    code = binHttp.GET();
    if (code != 200)
    {
        LOGF("[OTA] Download failed: HTTP %d\n", code);
        binHttp.end();
        otaRunning = false;
        vTaskDelete(nullptr);
        return;
    }

    int totalSize = binHttp.getSize();
    if (totalSize <= 0)
    {
        LOG("[OTA] Invalid content-length");
        binHttp.end();
        otaRunning = false;
        vTaskDelete(nullptr);
        return;
    }

    if (!Update.begin(totalSize))
    {
        LOG("[OTA] Not enough flash space");
        Update.printError(Serial);
        binHttp.end();
        otaRunning = false;
        vTaskDelete(nullptr);
        return;
    }

    size_t written = Update.writeStream(*binHttp.getStreamPtr());

    if (written != (size_t)totalSize)
    {
        LOGF("[OTA] Size mismatch: wrote %u of %d\n", written, totalSize);
        binHttp.end();
        otaRunning = false;
        vTaskDelete(nullptr);
        return;
    }

    if (Update.end() && Update.isFinished())
    {
        LOGF("[OTA] Flashed %u bytes — rebooting\n", written);
        binHttp.end();
        delay(500);
        ESP.restart();
    }
    else
    {
        LOG("[OTA] Flash failed");
        Update.printError(Serial);
    }

    binHttp.end();
    otaRunning = false;
    vTaskDelete(nullptr);
}

// ── Shared task launcher ──────────────────────────────────────────────────────
static void launchTask()
{
    otaRunning = true;
    if (xTaskCreate(otaTask, "ota_pull",
                    CFG_OTA_TASK_STACK, nullptr, 1, nullptr) != pdPASS)
    {
        LOG("[OTA] Task creation failed");
        otaRunning = false;
    }
}

// ── Public API ────────────────────────────────────────────────────────────────
void ota_pull_init(OtaMode mode)
{
    otaMode = mode;
    lastCheck = 0;
    forceCheck = false;
    forceFallback = false;
    otaRunning = false;
    LOGF("[OTA] Pull OTA ready — %s mode\n",
         mode == OtaMode::AUTO ? "AUTO" : "TIMER");
}

void ota_pull_force()
{
    if (otaRunning)
        return;
    forceFallback = false;
    forceCheck = true;
    LOG("[OTA] Force check scheduled (primary)");
}

void ota_pull_force_fallback()
{
    if (otaRunning)
        return;
    forceFallback = true;
    forceCheck = true;
    LOG("[OTA] Force check scheduled (fallback public repo)");
}

void ota_pull_handle()
{
    if (otaRunning)
        return;

    bool shouldRun = false;

    if (forceCheck)
    {
        forceCheck = false;
        shouldRun = true;
    }
    else if (otaMode == OtaMode::TIMER)
    {
        shouldRun = ((millis() - lastCheck) > CFG_OTA_CHECK_MS);
    }

    if (shouldRun)
    {
        lastCheck = millis();
        launchTask();
    }
}
