#include "local_control.h"
#include "Network/network.h"
#include "Network/mqtt/mqtt_trigger.h"
#include "Build/Log/cmd_registry.h"
#include "Build/Log/logger.h"
#include "Build/config/target_config.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <esp_system.h>
#include <cctype>

#if defined(FEATURE_MOTOR_V2)
#include "Motor/V2/motor_v2.h"
#elif defined(FEATURE_SERVO)
#include "Motor/servos.h"
#endif

#ifndef CFG_LOCAL_CONTROL_PORT
#define CFG_LOCAL_CONTROL_PORT 7777
#endif

#ifndef CFG_LOCAL_CONTROL_MAX_LINE
#define CFG_LOCAL_CONTROL_MAX_LINE 512
#endif

#ifndef CFG_LOCAL_CONTROL_LOG_CLIENTS
#define CFG_LOCAL_CONTROL_LOG_CLIENTS 0
#endif

static WiFiServer s_server(CFG_LOCAL_CONTROL_PORT);
static WiFiClient s_client;
static bool s_started = false;
static String s_line;
static String s_nonce;

static void appendJsonEscaped(String &out, const String &value)
{
    for (size_t i = 0; i < value.length(); i++)
    {
        const char ch = value[i];
        switch (ch)
        {
        case '\\':
        case '"':
            out += '\\';
            out += ch;
            break;
        case '\n':
            out += "\\n";
            break;
        case '\r':
            out += "\\r";
            break;
        case '\t':
            out += "\\t";
            break;
        default:
            out += ch;
            break;
        }
    }
}

static String jsonString(const char *key, const String &value)
{
    String out = "\"";
    out += key;
    out += "\":\"";
    appendJsonEscaped(out, value);
    out += "\"";
    return out;
}

static String makeNonce()
{
    const uint64_t mac = ESP.getEfuseMac();
    return String((uint32_t)(mac & 0xFFFFFFFF), HEX) + "-" +
           String(millis(), HEX) + "-" +
           String((uint32_t)esp_random(), HEX);
}

static bool jsonStringField(const String &json, const char *key, String &out)
{
    String needle = String("\"") + key + "\"";
    int pos = json.indexOf(needle);
    if (pos < 0)
        return false;
    pos = json.indexOf(':', pos + needle.length());
    if (pos < 0)
        return false;
    pos++;
    while (pos < (int)json.length() && isspace((unsigned char)json[pos]))
        pos++;
    if (pos >= (int)json.length() || json[pos] != '"')
        return false;
    pos++;

    out = "";
    bool escaped = false;
    for (; pos < (int)json.length(); pos++)
    {
        const char ch = json[pos];
        if (escaped)
        {
            switch (ch)
            {
            case 'n':
                out += '\n';
                break;
            case 'r':
                out += '\r';
                break;
            case 't':
                out += '\t';
                break;
            default:
                out += ch;
                break;
            }
            escaped = false;
            continue;
        }
        if (ch == '\\')
        {
            escaped = true;
            continue;
        }
        if (ch == '"')
            return true;
        out += ch;
    }
    return false;
}

static bool jsonIntField(const String &json, const char *key, int32_t &out)
{
    String needle = String("\"") + key + "\"";
    int pos = json.indexOf(needle);
    if (pos < 0)
        return false;
    pos = json.indexOf(':', pos + needle.length());
    if (pos < 0)
        return false;
    pos++;
    while (pos < (int)json.length() && isspace((unsigned char)json[pos]))
        pos++;

    bool negative = false;
    if (pos < (int)json.length() && json[pos] == '-')
    {
        negative = true;
        pos++;
    }
    if (pos >= (int)json.length() || !isdigit((unsigned char)json[pos]))
        return false;

    int32_t value = 0;
    while (pos < (int)json.length() && isdigit((unsigned char)json[pos]))
    {
        value = (value * 10) + (json[pos] - '0');
        pos++;
    }
    out = negative ? -value : value;
    return true;
}

static void sendLine(const String &line)
{
    if (s_client && s_client.connected())
        s_client.println(line);
}

static void sendHello()
{
    if (s_nonce.length() == 0)
        s_nonce = makeNonce();

    String hello;
    hello.reserve(260);
    hello += "{\"v\":1,\"type\":\"hello\",";
    hello += jsonString("root", CFG_MQTT_TOPIC_ROOT);
    hello += ",";
    hello += jsonString("client", CFG_MQTT_CLIENT);
    hello += ",";
    hello += jsonString("target", CFG_TARGET_NAME);
    hello += ",";
    hello += jsonString("ip", WiFi.localIP().toString());
    hello += ",";
    hello += jsonString("ssid", WiFi.SSID());
    hello += ",";
    hello += jsonString("nonce", s_nonce);
    hello += ",\"mqtt\":";
    hello += mqtt_trigger_connected() ? "1" : "0";
    hello += ",\"hotspot\":";
    hello += network_is_hotspot() ? "1" : "0";
    hello += "}";
    sendLine(hello);
}

static void handleLinkJson(const String &payload)
{
    String root;
    String nonce;
    String hub;
    String host;
    String sig;
    int32_t portValue = 0;
    int32_t priority = 0;
    int32_t ttl = 0;

    if (!jsonStringField(payload, "root", root) ||
        !jsonStringField(payload, "nonce", nonce) ||
        !jsonStringField(payload, "hub", hub) ||
        !jsonStringField(payload, "host", host) ||
        !jsonStringField(payload, "sig", sig) ||
        !jsonIntField(payload, "port", portValue))
    {
        sendLine("ERR malformed link");
        return;
    }

    jsonIntField(payload, "priority", priority);
    jsonIntField(payload, "ttl", ttl);

    if (nonce != s_nonce)
    {
        sendLine("ERR stale nonce");
        return;
    }
    if (portValue < 1 || portValue > 65535 || ttl < 0)
    {
        sendLine("ERR invalid link");
        return;
    }

    const bool ok = mqtt_trigger_apply_signed_link(root,
                                                   nonce,
                                                   hub,
                                                   host,
                                                   (uint16_t)portValue,
                                                   (int)priority,
                                                   (uint32_t)ttl,
                                                   sig);
    sendLine(ok ? "OK link" : "ERR link rejected");
    if (ok)
        s_nonce = makeNonce();
}

static void dispatchCommand(const String &cmd)
{
    if (cmd.length() == 0)
    {
        sendLine("ERR empty command");
        return;
    }

    if (cmd_dispatch(cmd))
        sendLine("OK cmd");
    else
        sendLine("ERR unknown command");
}

static void handleLine(String line)
{
    line.trim();
    if (line.length() == 0)
        return;

    if (line == "hello" || line == "status")
    {
        sendHello();
        if (line == "status")
            dispatchCommand("status");
        return;
    }

    if (line.startsWith("cmd "))
    {
        dispatchCommand(line.substring(4));
        return;
    }

#if defined(FEATURE_MOTOR_V2)
    if (line.startsWith("motion "))
    {
        motor_v2_handle_motion_json(line.substring(7));
        sendLine("OK motion");
        return;
    }
    if (line.startsWith("heartbeat "))
    {
        motor_v2_handle_heartbeat_json(line.substring(10));
        sendLine("OK heartbeat");
        return;
    }
    if (line.startsWith("motor "))
    {
        motor_v2_handle_command(line.substring(6));
        sendLine("OK motor");
        return;
    }
#elif defined(FEATURE_SERVO)
    if (line.startsWith("motion "))
    {
        servos_handle_motion_json(line.substring(7));
        sendLine("OK motion");
        return;
    }
    if (line.startsWith("heartbeat "))
    {
        servos_handle_heartbeat_json(line.substring(10));
        sendLine("OK heartbeat");
        return;
    }
#endif

    if (line.startsWith("link "))
    {
        handleLinkJson(line.substring(5));
        return;
    }

    dispatchCommand(line);
}

static void stopServer()
{
    if (s_client)
        s_client.stop();
    if (s_started)
    {
        s_server.end();
        s_started = false;
        LOG("[LOCAL] control port stopped");
    }
}

static void ensureServer()
{
    if (!network_connected())
    {
        stopServer();
        return;
    }

    if (!s_started)
    {
        s_server.begin();
        s_started = true;
        s_line = "";
        s_nonce = makeNonce();
        LOGF("[LOCAL] control/link port listening on %u\n", CFG_LOCAL_CONTROL_PORT);
    }
}

void local_control_init()
{
    ensureServer();
}

void local_control_handle()
{
    ensureServer();
    if (!s_started)
        return;

    if (!s_client || !s_client.connected())
    {
        WiFiClient next = s_server.available();
        if (next)
        {
            if (s_client)
                s_client.stop();
            s_client = next;
            s_line = "";
            s_nonce = makeNonce();
            sendHello();
#if CFG_LOCAL_CONTROL_LOG_CLIENTS
            LOG("[LOCAL] client connected");
#endif
        }
        return;
    }

    while (s_client.available())
    {
        char ch = (char)s_client.read();
        if (ch == '\r')
            continue;
        if (ch == '\n')
        {
            handleLine(s_line);
            s_line = "";
            continue;
        }

        if (s_line.length() >= CFG_LOCAL_CONTROL_MAX_LINE)
        {
            s_line = "";
            sendLine("ERR line too long");
            continue;
        }
        s_line += ch;
    }
}
