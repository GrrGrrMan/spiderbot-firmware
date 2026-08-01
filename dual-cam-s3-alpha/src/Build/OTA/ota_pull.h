#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// ota_pull.h — pull-OTA subsystem.
//
// Two firmware sources are supported:
//
//   PRIMARY   Private GitHub repo.  All requests carry GITHUB_TOKEN.
//             Triggered automatically on interval (TIMER mode) or by
//             ota_pull_force() / MQTT "ota" command.
//
//   FALLBACK  Public GitHub repo.  No auth token sent — works even when
//             the primary token has been rotated or invalidated.
//             Triggered manually only: ota_pull_force_fallback() / MQTT
//             "ota:fallback" command.
//             Firmware must be pushed to the fallback repo manually before
//             issuing this command.
//
// Only one OTA task runs at a time.  A fallback request while a primary
// download is in progress is silently dropped (try again after it finishes).
// ─────────────────────────────────────────────────────────────────────────────

enum class OtaMode
{
    TIMER, // scheduled interval + force trigger
    AUTO   // force trigger only, no interval
};

void ota_pull_init(OtaMode mode = OtaMode::TIMER);
void ota_pull_handle();
bool ota_pull_in_progress();

// Trigger an immediate check on the PRIMARY repo.
void ota_pull_force();

// Trigger an immediate check on the FALLBACK public repo (no auth required).
void ota_pull_force_fallback();