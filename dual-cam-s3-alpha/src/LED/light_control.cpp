#ifdef FEATURE_LIGHTS

#include "light_control.h"
#include "Build/config/target_config.h"
#include "Build/Log/cmd_registry.h"
#include "Build/Log/logger.h"
#include "Motor/servos.h"
#include "Network/mqtt/mqtt_trigger.h"
#include <math.h>

enum class LightKind : uint8_t
{
    PCA,
    GPIO
};

enum class LightEffect : uint8_t
{
    SOLID,
    OFF,
    BLINK,
    BREATHE,
    PULSE,
    TEST_RAMP
};

struct LightState
{
    bool on = false;
    uint8_t brightness = 0;
    uint8_t current = 0;
    uint8_t from = 0;
    LightEffect effect = LightEffect::OFF;
    uint8_t speed = 80;
    uint16_t transitionMs = 0;
    uint32_t effectStartMs = 0;
    uint32_t transitionStartMs = 0;
    bool transitioning = false;
};

struct LogicalLight
{
    const char *name;
    LightKind kind;
    uint8_t target;
    LightState state;
};

static LogicalLight s_lights[] = {
    {"status", LightKind::GPIO, CFG_ULTRASOUND_LED_PIN, {}},
    {"eye_left", LightKind::PCA, 0, {}},
    {"eye_right", LightKind::PCA, 1, {}},
    {"pca_ch0", LightKind::PCA, 0, {}},
    {"pca_ch1", LightKind::PCA, 1, {}},
    {"pca_ch2", LightKind::PCA, 2, {}},
    {"pca_ch3", LightKind::PCA, 3, {}},
    {"gpio35", LightKind::GPIO, CFG_ULTRASOUND_LED_PIN, {}},
};

static constexpr uint8_t kLightCount = sizeof(s_lights) / sizeof(s_lights[0]);

static const char *effectName(LightEffect effect)
{
    switch (effect)
    {
    case LightEffect::SOLID:
        return "solid";
    case LightEffect::OFF:
        return "off";
    case LightEffect::BLINK:
        return "blink";
    case LightEffect::BREATHE:
        return "breathe";
    case LightEffect::PULSE:
        return "pulse";
    case LightEffect::TEST_RAMP:
        return "test_ramp";
    }
    return "solid";
}

static LightEffect parseEffect(const String &raw, bool on)
{
    String value = raw;
    value.toLowerCase();
    if (!on || value == "off")
        return LightEffect::OFF;
    if (value == "blink")
        return LightEffect::BLINK;
    if (value == "breathe")
        return LightEffect::BREATHE;
    if (value == "pulse")
        return LightEffect::PULSE;
    if (value == "test_ramp")
        return LightEffect::TEST_RAMP;
    return LightEffect::SOLID;
}

static LogicalLight *findLight(const String &name)
{
    for (uint8_t i = 0; i < kLightCount; i++)
    {
        if (name == s_lights[i].name)
            return &s_lights[i];
    }
    return nullptr;
}

static uint16_t brightnessToPcaDuty(uint8_t brightness)
{
    if (brightness == 0)
        return 0;
    return (uint16_t)(((uint32_t)brightness * 4095UL) / 255UL);
}

static bool writeLight(LogicalLight &light, uint8_t brightness)
{
    brightness = constrain(brightness, 0, 255);
    bool ok = true;

    if (light.kind == LightKind::PCA)
        ok = pca9685_set_pwm_duty(light.target, brightnessToPcaDuty(brightness));
    else
    {
        pinMode(light.target, OUTPUT);
        analogWrite(light.target, brightness);
    }

    if (ok)
        light.state.current = brightness;
    return ok;
}

static int jsonValueStart(const String &payload, const char *key)
{
    String token = "\"" + String(key) + "\"";
    int keyPos = payload.indexOf(token);
    if (keyPos == -1)
        return -1;
    int colon = payload.indexOf(':', keyPos + token.length());
    if (colon == -1)
        return -1;
    int start = colon + 1;
    while (start < (int)payload.length() && isspace(payload[start]))
        start++;
    return start;
}

static bool jsonBool(const String &payload, const char *key, bool &out)
{
    int start = jsonValueStart(payload, key);
    if (start == -1)
        return false;
    if (payload.startsWith("true", start))
    {
        out = true;
        return true;
    }
    if (payload.startsWith("false", start))
    {
        out = false;
        return true;
    }
    return false;
}

static bool jsonInt(const String &payload, const char *key, long &out)
{
    int start = jsonValueStart(payload, key);
    if (start == -1)
        return false;
    int end = start;
    if (end < (int)payload.length() && payload[end] == '-')
        end++;
    while (end < (int)payload.length() && isdigit(payload[end]))
        end++;
    if (end <= start)
        return false;
    out = payload.substring(start, end).toInt();
    return true;
}

static bool jsonString(const String &payload, const char *key, String &out)
{
    int start = jsonValueStart(payload, key);
    if (start == -1 || start >= (int)payload.length() || payload[start] != '"')
        return false;
    int end = payload.indexOf('"', start + 1);
    if (end == -1)
        return false;
    out = payload.substring(start + 1, end);
    return true;
}

static uint32_t speedToPeriodMs(uint8_t speed, uint16_t slowMs, uint16_t fastMs)
{
    return slowMs - (((uint32_t)(slowMs - fastMs) * constrain(speed, 0, 255)) / 255UL);
}

static void publishState(const LogicalLight &light)
{
    String topic = String(CFG_LIGHT_TOPIC_PREFIX) + "/" + light.name + "/state";
    String payload = "{\"name\":\"" + String(light.name) +
                     "\",\"on\":" + String(light.state.on ? "true" : "false") +
                     ",\"brightness\":" + String(light.state.brightness) +
                     ",\"current\":" + String(light.state.current) +
                     ",\"effect\":\"" + String(effectName(light.state.effect)) + "\"" +
                     ",\"kind\":\"" + String(light.kind == LightKind::PCA ? "pca" : "gpio") + "\"" +
                     ",\"target\":" + String(light.target) + "}";
    mqtt_publish(topic.c_str(), payload);
}

static void applyState(LogicalLight &light, const String &payload)
{
    bool on = true;
    long brightness = 255;
    long speed = 80;
    long transitionMs = 0;
    String effect = "solid";

    jsonBool(payload, "on", on);
    jsonInt(payload, "brightness", brightness);
    jsonInt(payload, "bri", brightness);
    jsonInt(payload, "speed", speed);
    jsonInt(payload, "transition_ms", transitionMs);
    jsonString(payload, "effect", effect);

    brightness = constrain(brightness, 0L, 255L);
    speed = constrain(speed, 0L, 255L);
    transitionMs = constrain(transitionMs, 0L, 10000L);

    LightState &state = light.state;
    state.on = on && brightness > 0;
    state.brightness = state.on ? (uint8_t)brightness : 0;
    state.speed = (uint8_t)speed;
    state.transitionMs = (uint16_t)transitionMs;
    state.effect = parseEffect(effect, state.on);
    state.effectStartMs = millis();
    state.from = state.current;
    state.transitionStartMs = state.effectStartMs;
    state.transitioning = state.transitionMs > 0 &&
                          (state.effect == LightEffect::SOLID || state.effect == LightEffect::OFF);

    bool ok = true;
    if (state.effect == LightEffect::SOLID || state.effect == LightEffect::OFF)
    {
        if (!state.transitioning)
            ok = writeLight(light, state.brightness);
    }
    else
        ok = writeLight(light, 0);

    if (!ok)
        LOG("[LIGHT] " + String(light.name) + " unavailable; check PCA/GPIO wiring");
    else
        LOG("[LIGHT] " + String(light.name) + " <- " + String(effectName(state.effect)) +
            " brightness " + String(state.brightness));
    publishState(light);
}

static void applyAll(const String &payload)
{
    for (uint8_t i = 0; i < kLightCount; i++)
        applyState(s_lights[i], payload);
}

void light_init()
{
    LOG("[LIGHT] Ready | topic " + String(CFG_LIGHT_TOPIC_PREFIX) +
        " | PCA frequency follows servo setting " + String(CFG_SERVO_FREQ_HZ) + " Hz");
}

void light_publish_capabilities()
{
    String payload = "{\"lights\":[\"status\",\"eye_left\",\"eye_right\",\"pca_ch0\",\"pca_ch1\",\"pca_ch2\",\"pca_ch3\",\"gpio35\"],";
    payload += "\"effects\":[\"solid\",\"off\",\"blink\",\"breathe\",\"pulse\",\"test_ramp\"],";
    payload += "\"pca_hz\":" + String(CFG_SERVO_FREQ_HZ) + "}";
    mqtt_publish(CFG_LIGHT_CAPABILITIES_TOPIC, payload);
    LOG("[LIGHT] Capabilities published");
}

void light_publish_health()
{
    String payload = "{\"pca\":" + String(servos_found() ? 1 : 0) +
                     ",\"pca_hz\":" + String(CFG_SERVO_FREQ_HZ) +
                     ",\"lights\":" + String(kLightCount) +
                     ",\"heap\":" + String(ESP.getFreeHeap()) + "}";
    mqtt_publish(CFG_LIGHT_HEALTH_TOPIC, payload);
    LOG("[LIGHT] Health " + payload);
}

void light_handle()
{
    uint32_t now = millis();
    for (uint8_t i = 0; i < kLightCount; i++)
    {
        LogicalLight &light = s_lights[i];
        LightState &state = light.state;

        if (state.transitioning)
        {
            uint32_t elapsed = now - state.transitionStartMs;
            if (elapsed >= state.transitionMs)
            {
                writeLight(light, state.brightness);
                state.transitioning = false;
                publishState(light);
            }
            else
            {
                uint8_t next = state.from + (int16_t)(state.brightness - state.from) *
                                                (int32_t)elapsed / (int32_t)state.transitionMs;
                writeLight(light, next);
            }
            continue;
        }

        if (!state.on)
            continue;

        uint8_t next = state.current;
        switch (state.effect)
        {
        case LightEffect::BLINK:
        {
            uint32_t period = speedToPeriodMs(state.speed, 1600, 160);
            next = (((now - state.effectStartMs) % period) < (period / 2)) ? state.brightness : 0;
            break;
        }
        case LightEffect::BREATHE:
        case LightEffect::PULSE:
        {
            uint32_t period = speedToPeriodMs(state.speed, 5000, 700);
            float t = (float)((now - state.effectStartMs) % period) / (float)period;
            float wave = (state.effect == LightEffect::PULSE)
                             ? (1.0f - fabsf((t * 2.0f) - 1.0f))
                             : ((sinf((t * 2.0f * (float)M_PI) - ((float)M_PI / 2.0f)) + 1.0f) * 0.5f);
            next = (uint8_t)(state.brightness * wave);
            break;
        }
        case LightEffect::TEST_RAMP:
        {
            uint32_t period = speedToPeriodMs(state.speed, 4000, 500);
            float t = (float)((now - state.effectStartMs) % period) / (float)period;
            next = (uint8_t)(state.brightness * t);
            break;
        }
        default:
            continue;
        }

        if (next != state.current)
            writeLight(light, next);
    }
}

bool light_handle_mqtt_topic(const String &topic, const String &payload)
{
    String prefix = String(CFG_LIGHT_TOPIC_PREFIX) + "/";
    if (!topic.startsWith(prefix))
        return false;

    String rest = topic.substring(prefix.length());
    if (rest == "all/set")
    {
        applyAll(payload);
        return true;
    }

    const String suffix = "/set";
    if (!rest.endsWith(suffix))
        return true;

    String name = rest.substring(0, rest.length() - suffix.length());
    LogicalLight *light = findLight(name);
    if (!light)
    {
        LOG("[LIGHT] Unknown light '" + name + "'");
        return true;
    }

    applyState(*light, payload);
    return true;
}

static void lightCmdHandler(const String &msg)
{
    int first = msg.indexOf(':');
    int second = msg.indexOf(':', first + 1);
    String action = second == -1 ? msg.substring(first + 1)
                                 : msg.substring(first + 1, second);

    if (action == "health")
    {
        light_publish_health();
        return;
    }
    if (action == "capabilities")
    {
        light_publish_capabilities();
        return;
    }
    if (action == "all")
    {
        if (second == -1)
        {
            LOG("[LIGHT] Usage: light:all:{json}");
            return;
        }
        applyAll(msg.substring(second + 1));
        return;
    }

    LogicalLight *light = findLight(action);
    if (!light)
    {
        LOG("[LIGHT] Unknown. Use light:<name>:{json} | light:all:{json} | light:health | light:capabilities");
        return;
    }
    if (second == -1)
    {
        publishState(*light);
        return;
    }
    applyState(*light, msg.substring(second + 1));
}

void light_register_commands()
{
    cmd_register("light:", lightCmdHandler);
}

#endif // FEATURE_LIGHTS
