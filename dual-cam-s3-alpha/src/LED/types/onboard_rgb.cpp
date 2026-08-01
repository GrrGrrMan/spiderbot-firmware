#include "onboard_rgb.h"
#include "Build/config/target_config.h"
#include <Arduino.h>
#include "esp32-hal-rgb-led.h"

#if CFG_ONBOARD_RGB_DISABLE && (CFG_ONBOARD_RGB_PIN >= 0)
static uint8_t s_offRepeatsRemaining = CFG_ONBOARD_RGB_OFF_REPEAT_COUNT;
static uint32_t s_lastOffMs = 0;

void onboard_rgb_off()
{
    neopixelWrite((uint8_t)CFG_ONBOARD_RGB_PIN, 0, 0, 0);
    pinMode(CFG_ONBOARD_RGB_PIN, OUTPUT);
    digitalWrite(CFG_ONBOARD_RGB_PIN, LOW);
}

void onboard_rgb_init()
{
    onboard_rgb_off();
    s_lastOffMs = millis();
    s_offRepeatsRemaining = CFG_ONBOARD_RGB_OFF_REPEAT_COUNT;
}

void onboard_rgb_handle()
{
    if (s_offRepeatsRemaining == 0)
        return;

    uint32_t now = millis();
    if ((now - s_lastOffMs) < CFG_ONBOARD_RGB_OFF_REPEAT_MS)
        return;

    s_lastOffMs = now;
    s_offRepeatsRemaining--;
    onboard_rgb_off();
}
#else
void onboard_rgb_off() {}
void onboard_rgb_init() {}
void onboard_rgb_handle() {}
#endif
