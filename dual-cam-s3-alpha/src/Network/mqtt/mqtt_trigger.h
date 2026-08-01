#pragma once
#include <Arduino.h>

// mqtt_trigger.h - MQTT connection lifecycle and command dispatch.
//
// This module owns:
//   - WiFi+MQTT connection and reconnect logic
//   - Routing inbound messages through cmd_registry or subsystem handlers
//   - Registering itself as the log sink so LOG() forwards over MQTT
//   - Built-in commands: gpio:, status, reset, ota
//
// Topic layout:
//   legacy: beanspiderbot/cmd and beanspiderbot/log
//   v2:     <root>/cmd/discrete, <root>/cmd/motion, <root>/cmd/motor,
//           <root>/controller/heartbeat, <root>/motor/state, <root>/event,
//           <root>/log. Motor state is quiet unless requested by the motor
//           controller through "stream status" or "status".

void mqtt_trigger_init();
void mqtt_trigger_handle();
bool mqtt_trigger_connected();

// Apply a signed broker link offered over the local control port.
// The signature is HMAC-SHA256 over:
//   v1|<root>|<nonce>|<hub>|<host>|<port>|<priority>|<ttl>
// using MQTT_LINK_TOKEN from secrets.h.
bool mqtt_trigger_apply_signed_link(const String &root,
                                    const String &nonce,
                                    const String &hub,
                                    const String &host,
                                    uint16_t port,
                                    int priority,
                                    uint32_t ttl,
                                    const String &sig);

// Forward a message to MQTT. Called internally by the log sink and by
// subsystems that publish state/event updates. Prefer LOG() for plain text logs.
void mqtt_log(const String &msg);
void mqtt_publish(const char *topic, const String &payload);
