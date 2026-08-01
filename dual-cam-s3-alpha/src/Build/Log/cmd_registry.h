#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
// cmd_registry.h — lightweight prefix-based command dispatcher.
//
// Each subsystem registers its own handler during init:
//
//   cmd_register("led:",   led_cmd_handler);
//   cmd_register("servo:", servo_cmd_handler);
//   cmd_register("status", status_cmd_handler);  // exact match
//
// mqtt_trigger.cpp then dispatches every inbound CMD message with one call:
//
//   if (!cmd_dispatch(msg))
//       LOG("[CMD] Unknown: " + msg);
//
// Rules:
//  - Prefixes ending in ':' match any message that startsWith() that string.
//  - Prefixes with no ':' at the end match the message exactly.
//  - First registered match wins.
//  - Maximum CMD_REGISTRY_MAX entries (increase if needed).
// ─────────────────────────────────────────────────────────────────────────────

#define CMD_REGISTRY_MAX 24

typedef void (*CmdHandler)(const String &msg);

// Register a prefix (or exact keyword) and its handler.
// Safe to call from any module's init function before or after MQTT connects.
void cmd_register(const char *prefix, CmdHandler handler);

// Dispatch msg to the first matching handler.
// Returns true if a handler was found and called, false if no match.
bool cmd_dispatch(const String &msg);
