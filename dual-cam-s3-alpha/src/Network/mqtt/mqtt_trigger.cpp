#include "mqtt_trigger.h"
#include "Build/Log/cmd_registry.h"
#include "Build/config/target_config.h"
#include "Build/Log/logger.h"
#include "Build/Log/log_sink.h"
#include "Build/OTA/ota_pull.h"
#include "Build/config/secrets.h"
#include "Network/network.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <mbedtls/md.h>
#include <cstring>

#ifdef MQTT_USE_TLS
#include <WiFiClientSecure.h>
static WiFiClientSecure wifiClient;
#else
#include <WiFiClient.h>
static WiFiClient wifiClient;
#endif

#if defined(FEATURE_MOTOR_V3)
#include "Motor/V3/motor_v3.h"
#elif defined(FEATURE_MOTOR_V2)
#include "Motor/V2/motor_v2.h"
#elif defined(FEATURE_SERVO)
#include "Motor/servos.h"
#endif

#ifndef CFG_MQTT_LOG_LINK_REFRESH
#define CFG_MQTT_LOG_LINK_REFRESH 0
#endif

#ifdef FEATURE_LIGHTS
#include "LED/light_control.h"
#endif

static PubSubClient mqtt(wifiClient);
static String s_mqttClientId;
static uint32_t s_disconnectedSinceMs = 0;
static uint32_t s_lastRetryMs = 0;

enum class BrokerSlot : uint8_t
{
    Primary,
    Linked
};

static BrokerSlot s_activeBroker = BrokerSlot::Primary;
static String s_linkHost;
static uint16_t s_linkPort = 0;
static uint32_t s_linkUntilMs = 0;
static bool s_linkActive = false;

struct LinkOffer
{
    bool valid;
    int priority;
    uint16_t port;
    uint32_t ttl;
    String hub;
    String host;
};

struct LocalBrokerCandidate
{
    const char *label;
    const char *host;
    uint16_t port;
};

static const LocalBrokerCandidate kLocalBrokers[] = CFG_MQTT_LOCAL_BROKERS;
static const size_t kLocalBrokerCount = sizeof(kLocalBrokers) / sizeof(kLocalBrokers[0]);

static int s_selectedLocalBroker = -1;
static int s_activeLocalBroker = -1;

#ifdef MQTT_USERNAME
static const char *kMqttUsername = MQTT_USERNAME;
#else
static const char *kMqttUsername = nullptr;
#endif

#ifdef MQTT_PASSWORD
static const char *kMqttPassword = MQTT_PASSWORD;
#else
static const char *kMqttPassword = nullptr;
#endif

#ifdef MQTT_LINK_TOKEN
static const char *kMqttLinkToken = MQTT_LINK_TOKEN;
#else
static const char *kMqttLinkToken = nullptr;
#endif

#ifdef MQTT_LINK_USERNAME
static const char *kMqttLinkUsername = MQTT_LINK_USERNAME;
#elif defined(MQTT_USERNAME)
static const char *kMqttLinkUsername = MQTT_USERNAME;
#else
static const char *kMqttLinkUsername = nullptr;
#endif

#ifdef MQTT_LINK_PASSWORD
static const char *kMqttLinkPassword = MQTT_LINK_PASSWORD;
#elif defined(MQTT_PASSWORD)
static const char *kMqttLinkPassword = MQTT_PASSWORD;
#else
static const char *kMqttLinkPassword = nullptr;
#endif

static bool hasText(const char *value)
{
    return value && value[0] != '\0';
}

static String linkOfferCanonical(const String &root,
                                 const String &nonce,
                                 const String &hub,
                                 const String &host,
                                 uint16_t port,
                                 int priority,
                                 uint32_t ttl)
{
    return String("v1|") + root + "|" + nonce + "|" + hub + "|" + host + "|" +
           String(port) + "|" + String(priority) + "|" + String(ttl);
}

static String hmacSha256Hex(const String &payload)
{
    if (!hasText(kMqttLinkToken))
        return "";

    unsigned char digest[32] = {0};
    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);
    const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (!info ||
        mbedtls_md_setup(&ctx, info, 1) != 0 ||
        mbedtls_md_hmac_starts(&ctx,
                               reinterpret_cast<const unsigned char *>(kMqttLinkToken),
                               strlen(kMqttLinkToken)) != 0 ||
        mbedtls_md_hmac_update(&ctx,
                               reinterpret_cast<const unsigned char *>(payload.c_str()),
                               payload.length()) != 0 ||
        mbedtls_md_hmac_finish(&ctx, digest) != 0)
    {
        mbedtls_md_free(&ctx);
        return "";
    }
    mbedtls_md_free(&ctx);

    static const char hex[] = "0123456789abcdef";
    String out;
    out.reserve(64);
    for (uint8_t byte : digest)
    {
        out += hex[(byte >> 4) & 0x0F];
        out += hex[byte & 0x0F];
    }
    return out;
}

static bool secureEquals(const String &left, const String &right)
{
    if (left.length() != right.length())
        return false;

    uint8_t diff = 0;
    for (size_t i = 0; i < left.length(); i++)
        diff |= (uint8_t)(left[i] ^ right[i]);
    return diff == 0;
}

static bool localBrokerIndexValid(int index)
{
    return index >= 0 && (size_t)index < kLocalBrokerCount &&
           hasText(kLocalBrokers[index].host) && kLocalBrokers[index].port != 0;
}

static int preferredLocalBrokerIndex()
{
    if (localBrokerIndexValid(s_selectedLocalBroker))
        return s_selectedLocalBroker;
    if (localBrokerIndexValid(s_activeLocalBroker))
        return s_activeLocalBroker;
    return localBrokerIndexValid(0) ? 0 : -1;
}

static const char *localBrokerLabel(int index)
{
    return localBrokerIndexValid(index) && hasText(kLocalBrokers[index].label)
               ? kLocalBrokers[index].label
               : "local";
}

static const char *brokerLabel(BrokerSlot slot)
{
    switch (slot)
    {
    case BrokerSlot::Primary:
        return localBrokerLabel(preferredLocalBrokerIndex());
    case BrokerSlot::Linked:
        return "linked";
    }
    return "unknown";
}

static const char *brokerHost(BrokerSlot slot)
{
    switch (slot)
    {
    case BrokerSlot::Primary:
    {
        const int index = preferredLocalBrokerIndex();
        return localBrokerIndexValid(index) ? kLocalBrokers[index].host : CFG_MQTT_PRIMARY_HOST;
    }
    case BrokerSlot::Linked:
        return s_linkHost.c_str();
    }
    return CFG_MQTT_PRIMARY_HOST;
}

static uint16_t brokerPort(BrokerSlot slot)
{
    switch (slot)
    {
    case BrokerSlot::Primary:
    {
        const int index = preferredLocalBrokerIndex();
        return localBrokerIndexValid(index) ? kLocalBrokers[index].port : CFG_MQTT_PRIMARY_PORT;
    }
    case BrokerSlot::Linked:
        return s_linkPort;
    }
    return CFG_MQTT_PRIMARY_PORT;
}

static const char *brokerUsername(BrokerSlot slot)
{
    switch (slot)
    {
    case BrokerSlot::Primary:
        return kMqttUsername;
    case BrokerSlot::Linked:
        return kMqttLinkUsername;
    }
    return nullptr;
}

static const char *brokerPassword(BrokerSlot slot)
{
    switch (slot)
    {
    case BrokerSlot::Primary:
        return kMqttPassword;
    case BrokerSlot::Linked:
        return kMqttLinkPassword;
    }
    return nullptr;
}

static bool probeTcpBroker(const char *host, uint16_t port)
{
    if (!hasText(host) || port == 0)
        return false;

    WiFiClient probe;
    const bool ok = probe.connect(host, port, CFG_MQTT_LOCAL_BROKER_PROBE_MS);
    probe.stop();
    return ok;
}

static bool selectReachableLocalBroker(bool verbose)
{
    if (WiFi.status() != WL_CONNECTED || kLocalBrokerCount == 0 || !network_is_hotspot())
        return false;

    for (size_t i = 0; i < kLocalBrokerCount; i++)
    {
        if (!localBrokerIndexValid((int)i))
            continue;

        if (verbose)
        {
            LOGF("[MQTT] Probing local broker %s %s:%u...\n",
                 localBrokerLabel((int)i),
                 kLocalBrokers[i].host,
                 kLocalBrokers[i].port);
        }

        if (probeTcpBroker(kLocalBrokers[i].host, kLocalBrokers[i].port))
        {
            if (s_selectedLocalBroker != (int)i)
            {
                LOGF("[MQTT] Local broker selected: %s %s:%u\n",
                     localBrokerLabel((int)i),
                     kLocalBrokers[i].host,
                     kLocalBrokers[i].port);
            }
            s_selectedLocalBroker = (int)i;
            return true;
        }
    }

    s_selectedLocalBroker = -1;
    return false;
}

static bool linkExpired(uint32_t now)
{
    return s_linkActive && s_linkUntilMs != 0 &&
           (int32_t)(now - s_linkUntilMs) >= 0;
}

static void clearLink()
{
    s_linkActive = false;
    s_linkHost = "";
    s_linkPort = 0;
    s_linkUntilMs = 0;
}

static bool linkEnabled(uint32_t now)
{
    if (!s_linkActive)
        return false;
    if (linkExpired(now))
    {
        clearLink();
        return false;
    }
    return s_linkPort != 0 && s_linkHost.length() > 0;
}

static void configureTransport()
{
#ifdef MQTT_USE_TLS
    #ifdef MQTT_BROKER_CA
    wifiClient.setCACert(MQTT_BROKER_CA);
    #else
    wifiClient.setInsecure();
    LOG("[MQTT] TLS enabled without CA - using insecure validation");
    #endif
#endif
}

void mqtt_log(const String &msg)
{
    if (!mqtt.connected())
        return;

    String out = (msg.length() > CFG_MQTT_MAX_MSG)
                     ? msg.substring(0, CFG_MQTT_MAX_MSG)
                     : msg;
    mqtt.publish(CFG_MQTT_TOPIC_LOG, out.c_str());
    if (strcmp(CFG_MQTT_TOPIC_LOG, CFG_MQTT_TOPIC_LOG_V2) != 0)
        mqtt.publish(CFG_MQTT_TOPIC_LOG_V2, out.c_str());
}

void mqtt_publish(const char *topic, const String &payload)
{
    if (!mqtt.connected())
        return;

    if (payload.length() > CFG_MQTT_MAX_MSG)
    {
        LOGF("[MQTT] Publish skipped for %s: payload too large (%u > %u)\n",
             topic,
             (unsigned)payload.length(),
             (unsigned)CFG_MQTT_MAX_MSG);
        return;
    }

    if (!mqtt.publish(topic, payload.c_str()))
        LOGF("[MQTT] Publish failed for %s\n", topic);
}

static void applyLinkOffer(const LinkOffer &offer)
{
    const bool sameActiveLink = s_linkActive &&
                                s_linkHost == offer.host &&
                                s_linkPort == offer.port &&
                                s_activeBroker == BrokerSlot::Linked &&
                                mqtt.connected();

    s_linkHost = offer.host;
    s_linkPort = offer.port;
    s_linkActive = true;
    s_linkUntilMs = offer.ttl == 0 ? 0 : millis() + (offer.ttl * 1000UL);

    if (sameActiveLink)
    {
#if CFG_MQTT_LOG_LINK_REFRESH
        LOG("[MQTT] Link offer refreshed: " + offer.hub + " " +
            offer.host + ":" + String(offer.port));
#endif
        return;
    }

    LOG("[MQTT] Link offer selected: " + offer.hub + " " +
        offer.host + ":" + String(offer.port) +
        " priority:" + String(offer.priority));
    mqtt_publish(CFG_MQTT_TOPIC_EVENT,
                 "{\"type\":\"mqtt_link_offer\",\"detail\":\"switching to selected broker\"}");

    mqtt.disconnect();
    s_disconnectedSinceMs = 0;
    s_lastRetryMs = 0;
}

static void handleGpioCmd(const String &msg)
{
    int first = msg.indexOf(':');
    int second = msg.indexOf(':', first + 1);
    int third = msg.indexOf(':', second + 1);

    String action = msg.substring(first + 1, second);
    int pin = msg.substring(second + 1,
                            third == -1 ? (int)msg.length() : third)
                  .toInt();

    if (action == "read")
    {
        pinMode(pin, INPUT);
        LOG("[GPIO] pin " + String(pin) + " = " + String(digitalRead(pin)));
    }
    else if (action == "write" && third != -1)
    {
        int value = msg.substring(third + 1).toInt();
        pinMode(pin, OUTPUT);
        digitalWrite(pin, value ? HIGH : LOW);
        LOG("[GPIO] pin " + String(pin) + " set to " + String(value));
    }
    else if (action == "pwm" && third != -1)
    {
        int duty = msg.substring(third + 1).toInt();
        pinMode(pin, OUTPUT);
        analogWrite(pin, duty);
        LOG("[GPIO] pin " + String(pin) + " PWM duty " + String(duty));
    }
    else
    {
        LOG("[GPIO] Unknown - use gpio:read:<pin> | gpio:write:<pin>:<val> | gpio:pwm:<pin>:<duty>");
    }
}

static void handleStatusCmd(const String &)
{
    char buf[CFG_MQTT_MAX_MSG];
    snprintf(buf, sizeof(buf),
             "[STATUS] IP:%s heap:%u uptime:%lus root:%s",
             WiFi.localIP().toString().c_str(),
             ESP.getFreeHeap(),
             millis() / 1000UL,
             CFG_MQTT_TOPIC_ROOT);
    LOG(String(buf));
}

static void handleResetCmd(const String &)
{
    LOG("[CMD] Rebooting...");
    delay(500);
    ESP.restart();
}

static void handleOtaCmd(const String &)
{
    ota_pull_force();
    LOG("[OTA] Force triggered via MQTT (primary)");
}

static void handleOtaFallbackCmd(const String &)
{
    ota_pull_force_fallback();
    LOG("[OTA] Fallback triggered via MQTT (public repo - no token required)");
}

static void handleLinkCmd(const String &)
{
    LOG("[MQTT] Link command ignored - ESP links are accepted over the local control port");
}

static void handleUnlinkCmd(const String &)
{
    clearLink();
    LOG("[MQTT] Link broker cleared - returning to primary");
    mqtt_publish(CFG_MQTT_TOPIC_EVENT,
                 "{\"type\":\"mqtt_unlink\",\"detail\":\"returning to primary broker\"}");
    mqtt.disconnect();
    s_disconnectedSinceMs = 0;
    s_lastRetryMs = 0;
}

bool mqtt_trigger_apply_signed_link(const String &root,
                                    const String &nonce,
                                    const String &hub,
                                    const String &host,
                                    uint16_t port,
                                    int priority,
                                    uint32_t ttl,
                                    const String &sig)
{
    if (root != CFG_MQTT_TOPIC_ROOT)
    {
        LOG("[MQTT] Local link ignored - root mismatch");
        return false;
    }
    if (nonce.length() == 0 || host.length() == 0 || port == 0)
    {
        LOG("[MQTT] Local link ignored - missing nonce/host/port");
        return false;
    }
    if (!hasText(kMqttLinkToken))
    {
        LOG("[MQTT] Local link ignored - MQTT_LINK_TOKEN is not configured");
        return false;
    }

    const String canonical = linkOfferCanonical(root, nonce, hub, host, port, priority, ttl);
    const String expected = hmacSha256Hex(canonical);
    if (expected.length() == 0 || !secureEquals(sig, expected))
    {
        LOG("[MQTT] Local link ignored - bad signature");
        return false;
    }

    LinkOffer offer;
    offer.valid = true;
    offer.priority = priority;
    offer.port = port;
    offer.ttl = ttl;
    offer.hub = hub;
    offer.host = host;
    applyLinkOffer(offer);
    return true;
}

bool mqtt_trigger_connected()
{
    return mqtt.connected();
}

static void onMessage(char *topic, byte *payload, unsigned int length)
{
    if (length < 1)
        return;

    String msg;
    msg.reserve(length);
    for (unsigned int i = 0; i < length; i++)
        msg += (char)payload[i];

    String topicName(topic);

    if (topicName == CFG_MQTT_TOPIC_OTA)
    {
        if (msg == "1")
        {
            LOG("[MQTT] OTA trigger received");
            mqtt.publish(CFG_MQTT_TOPIC_OTA, "", true);
            ota_pull_force();
        }
        return;
    }

#ifdef FEATURE_LIGHTS
    if (light_handle_mqtt_topic(topicName, msg))
        return;
#endif

    if (topicName == CFG_MQTT_TOPIC_CMD || topicName == CFG_MQTT_TOPIC_CMD_DISCRETE)
    {
        LOG("[MQTT] CMD: " + msg);
        if (!cmd_dispatch(msg))
            LOG("[CMD] Unknown: " + msg);
        return;
    }

#if defined(FEATURE_MOTOR_V3)
    if (topicName == CFG_MQTT_TOPIC_CMD_MOTION)
    {
        // Route incoming streaming packets directly to the V3 Jitter Buffer
        motor_v3_handle_stream_json(msg);
        return;
    }
#elif defined(FEATURE_MOTOR_V2)
    if (topicName == CFG_MQTT_TOPIC_CMD_MOTOR)
    {
        motor_v2_handle_command(msg);
        return;
    }

    if (topicName == CFG_MQTT_TOPIC_CMD_MOTION)
    {
        motor_v2_handle_motion_json(msg);
        return;
    }

    if (topicName == CFG_MQTT_TOPIC_CONTROLLER_HEARTBEAT)
    {
        motor_v2_handle_heartbeat_json(msg);
        return;
    }
#elif defined(FEATURE_SERVO)
    if (topicName == CFG_MQTT_TOPIC_CMD_MOTION)
    {
        servos_handle_motion_json(msg);
        return;
    }

    if (topicName == CFG_MQTT_TOPIC_CONTROLLER_HEARTBEAT)
    {
        servos_handle_heartbeat_json(msg);
        return;
    }
#endif
}

static bool connectBroker(BrokerSlot slot)
{
    if (slot == BrokerSlot::Primary && !localBrokerIndexValid(s_selectedLocalBroker))
    {
        if (!selectReachableLocalBroker(false))
            return false;
    }

    const char *host = brokerHost(slot);
    const uint16_t port = brokerPort(slot);
    if (!hasText(host) || port == 0)
        return false;

    LOGF("[MQTT] Connecting to %s broker %s:%u...\n",
         brokerLabel(slot), host, port);

    const char *willPayload = "{\"type\":\"device_disconnect\",\"detail\":\"broker session lost\"}";
    bool ok = false;
    if (s_mqttClientId.isEmpty())
    {
        uint64_t mac = ESP.getEfuseMac();
        s_mqttClientId = String(CFG_MQTT_CLIENT) + "-" + String((uint32_t)(mac & 0xFFFFFF), HEX);
    }

    mqtt.setServer(host, port);

    const char *username = brokerUsername(slot);
    const char *password = brokerPassword(slot);
    if (hasText(username))
    {
        ok = mqtt.connect(s_mqttClientId.c_str(),
                          username,
                          password,
                          CFG_MQTT_TOPIC_EVENT,
                          0,
                          false,
                          willPayload);
    }
    else
    {
        ok = mqtt.connect(s_mqttClientId.c_str(),
                          CFG_MQTT_TOPIC_EVENT,
                          0,
                          false,
                          willPayload);
    }

    if (!ok)
    {
        LOGF("[MQTT] %s broker connect failed (rc=%d), will retry\n",
             brokerLabel(slot), mqtt.state());
        return false;
    }

    s_activeBroker = slot;
    s_disconnectedSinceMs = 0;
    if (slot == BrokerSlot::Primary)
    {
        s_activeLocalBroker = s_selectedLocalBroker;
    }
    LOGF("[MQTT] Connected to %s broker\n", brokerLabel(slot));
    mqtt.subscribe(CFG_MQTT_TOPIC_OTA);
    mqtt.subscribe(CFG_MQTT_TOPIC_CMD);
    mqtt.subscribe(CFG_MQTT_TOPIC_CMD_DISCRETE);
#ifdef FEATURE_LIGHTS
    {
        String lightOne = String(CFG_LIGHT_TOPIC_PREFIX) + "/+/set";
        String lightAll = String(CFG_LIGHT_TOPIC_PREFIX) + "/all/set";
        mqtt.subscribe(lightOne.c_str());
        mqtt.subscribe(lightAll.c_str());
    }
#endif
#if defined(FEATURE_MOTOR_V3)
    // V3 only needs the high-speed motion stream and heartbeat
    mqtt.subscribe(CFG_MQTT_TOPIC_CMD_MOTION);
    mqtt.subscribe(CFG_MQTT_TOPIC_CONTROLLER_HEARTBEAT);
#elif defined(FEATURE_MOTOR_V2)
    mqtt.subscribe(CFG_MQTT_TOPIC_CMD_MOTOR);
    mqtt.subscribe(CFG_MQTT_TOPIC_CMD_MOTION);
    mqtt.subscribe(CFG_MQTT_TOPIC_CONTROLLER_HEARTBEAT);
#elif defined(FEATURE_SERVO)
#endif

    char event[160];
    snprintf(event, sizeof(event),
             "{\"type\":\"online\",\"detail\":\"mqtt connected\",\"broker\":\"%s\"}",
             brokerLabel(slot));
    mqtt_publish(CFG_MQTT_TOPIC_EVENT, event);
    return true;
}

static bool connectLocalOrFallback(bool verbose)
{
    if (!network_is_hotspot())
    {
        static bool loggedWaitingForLocalLink = false;
        if (!loggedWaitingForLocalLink)
        {
            LOG("[MQTT] Normal WiFi active - waiting for signed local Pi link");
            loggedWaitingForLocalLink = true;
        }
        return false;
    }

    if (selectReachableLocalBroker(verbose) && connectBroker(BrokerSlot::Primary))
        return true;

    LOG("[MQTT] Hotspot broker is not reachable yet");
    return false;
}

static void reconnect()
{
    if (WiFi.status() != WL_CONNECTED || mqtt.connected())
        return;

    const uint32_t now = millis();
    if (s_disconnectedSinceMs == 0)
        s_disconnectedSinceMs = now;

    if (network_is_hotspot())
    {
        clearLink();
        if (connectLocalOrFallback(false))
            return;
    }

    if (linkEnabled(now))
    {
        if (connectBroker(BrokerSlot::Linked))
            return;

        LOG("[MQTT] Linked broker unavailable - clearing link");
        clearLink();
        s_disconnectedSinceMs = 0;
    }

    connectLocalOrFallback(false);
}

void mqtt_trigger_init()
{
    configureTransport();

    log_sink_register([](const String &msg)
                      { mqtt_log(msg); });

    cmd_register("gpio:", handleGpioCmd);
    cmd_register("status", handleStatusCmd);
    cmd_register("reset", handleResetCmd);
    cmd_register("ota", handleOtaCmd);
    cmd_register("ota:fallback", handleOtaFallbackCmd);
    cmd_register("link", handleLinkCmd);
    cmd_register("unlink", handleUnlinkCmd);

    mqtt.setBufferSize(CFG_MQTT_BUFFER_SIZE);
    mqtt.setKeepAlive(CFG_MQTT_KEEPALIVE_SEC);
    mqtt.setCallback(onMessage);
    reconnect();

    LOG("[MQTT] Trigger ready");
}

void mqtt_trigger_handle()
{
    static bool wasConnected = false;
    static bool wifiWasConnected = false;

    if (WiFi.status() != WL_CONNECTED)
    {
        wifiWasConnected = false;
        return;
    }

    if (!wifiWasConnected)
    {
        wifiWasConnected = true;
        s_disconnectedSinceMs = 0;
        s_lastRetryMs = 0;
    }

    if (!mqtt.connected())
    {
        if (wasConnected)
        {
            wasConnected = false;
            s_disconnectedSinceMs = millis();
            LOG("[MQTT] Connection lost");
        }

        if ((millis() - s_lastRetryMs) > CFG_MQTT_RETRY_MS)
        {
            s_lastRetryMs = millis();
            reconnect();
        }
        return;
    }

    wasConnected = true;
    mqtt.loop();

    if (s_activeBroker == BrokerSlot::Linked && linkExpired(millis()))
    {
        LOG("[MQTT] Link broker TTL expired - returning to primary");
        mqtt_publish(CFG_MQTT_TOPIC_EVENT,
                     "{\"type\":\"mqtt_link_expired\",\"detail\":\"returning to primary broker\"}");
        clearLink();
        mqtt.disconnect();
        s_disconnectedSinceMs = 0;
        s_lastRetryMs = 0;
        return;
    }

}
