#include "safeboot.h"
#include "Build/config/target_config.h"
#include "Build/Log/logger.h"
#include <Arduino.h>
#include <esp_system.h>

// ── RTC memory — persists across soft resets (panics, watchdog, ESP.restart)
//               — cleared on power-on or deep-sleep wake
RTC_DATA_ATTR static uint8_t s_crashCount    = 0;
RTC_DATA_ATTR static bool    s_safeMode      = false;

static bool    s_counterCleared = false;   // prevent double-clear in loop()
static uint32_t s_stableStart   = 0;

// ─────────────────────────────────────────────────────────────────────────────
bool safeboot_check()
{
    esp_reset_reason_t reason = esp_reset_reason();

    // Crashes that can trap the device in a boot loop:
    bool wasCrash = (reason == ESP_RST_PANIC  ||
                     reason == ESP_RST_WDT    ||
                     reason == ESP_RST_INT_WDT ||
                     reason == ESP_RST_TASK_WDT);

    if (wasCrash)
    {
        s_crashCount++;
        LOGF("[SAFEBOOT] Crash reset detected — consecutive crashes: %u / %u\n",
             s_crashCount, (uint8_t)CFG_SAFEBOOT_THRESHOLD);
    }
    else
    {
        // Power-on or normal software reset (OTA reboot, ESP.restart() after
        // a successful flash).  If we're not already stuck in safe mode, this
        // clean boot resets the panic counter.
        if (!s_safeMode)
            s_crashCount = 0;

        LOGF("[SAFEBOOT] Clean reset (%s) — crash count: %u\n",
             reason == ESP_RST_SW      ? "SW"      :
             reason == ESP_RST_POWERON ? "POWERON" : "OTHER",
             s_crashCount);
    }

    // Engage safe mode once threshold is reached — stays active until a
    // clean firmware flash delivers a boot that completes without panic.
    if (!s_safeMode && s_crashCount >= CFG_SAFEBOOT_THRESHOLD)
    {
        s_safeMode = true;
        LOG("[SAFEBOOT] *** SAFE MODE ACTIVE ***");
        LOG("[SAFEBOOT] All drivers skipped. Flash new firmware via OTA.");
        LOG("[SAFEBOOT] Or send MQTT 'reset' after fixing the firmware.");
    }

    if (s_safeMode)
    {
        LOGF("[SAFEBOOT] Running in safe mode (crash count was %u)\n",
             s_crashCount);
    }

    s_stableStart = millis();
    return s_safeMode;
}

// ─────────────────────────────────────────────────────────────────────────────
void safeboot_update()
{
    if (s_safeMode || s_counterCleared) return;

    if ((millis() - s_stableStart) >= CFG_SAFEBOOT_STABLE_MS)
    {
        s_counterCleared = true;
        s_crashCount     = 0;
        s_safeMode       = false;
        LOGF("[SAFEBOOT] Device stable for %u s — crash counter cleared.\n",
             CFG_SAFEBOOT_STABLE_MS / 1000);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
bool safeboot_active()
{
    return s_safeMode;
}
