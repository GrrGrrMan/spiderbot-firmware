#pragma once
#include <Arduino.h>

#ifndef CFG_LIGHT_TOPIC_PREFIX
#define CFG_LIGHT_TOPIC_PREFIX "spiderbot/s3/lights"
#endif
#ifndef CFG_LIGHT_HEALTH_TOPIC
#define CFG_LIGHT_HEALTH_TOPIC "spiderbot/s3/health"
#endif
#ifndef CFG_LIGHT_CAPABILITIES_TOPIC
#define CFG_LIGHT_CAPABILITIES_TOPIC "spiderbot/s3/capabilities"
#endif

void light_init();
void light_handle();
void light_register_commands();
bool light_handle_mqtt_topic(const String &topic, const String &payload);
void light_publish_health();
void light_publish_capabilities();
