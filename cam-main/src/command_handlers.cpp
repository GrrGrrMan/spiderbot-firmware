#include "command_handlers.h"
#include "cmd_schema.h"
#include "logger.h"

void registerAllCommandHandlers(
    CommandDispatcher& dispatcher,
    OTAManager& otaMgr,
    MQTTManager& mqttMgr
) {
    // 1. System Logging & Config Handshake
    dispatcher.registerHandler(CMD_TYPE_SYSTEM, [&mqttMgr](const JsonDocument& doc) {
        if (doc["logging"].is<bool>()) {
            g_logEnabled = doc["logging"].as<bool>();
            LOG_SYS("Logging state: %d", g_logEnabled);
        }
        if (doc["command"].is<const char*>()) {
            String cmd = doc["command"].as<String>();
            if (cmd == "get_config") {
                mqttMgr.sendConfig();
                LOG_SYS("Configuration handshake published on request.");
            }
        }
    });

    // 2. OTA Firmware Update Handler
    dispatcher.registerHandler(CMD_TYPE_OTA, [&otaMgr](const JsonDocument& doc) {
        bool forceFallback = doc["fallback"]     | false;
        bool forcePrimary  = doc["primary"]      | false;

        String customOwner = doc["owner"]        | "";
        String customRepo  = doc["repo"]         | "";
        String customBranch= doc["branch"]       | "";
        String customPath  = doc["project_path"] | "";
        String customPat   = doc["pat"]          | "";

        LOG_SYS("Remote OTA command received via MQTT!");
        otaMgr.checkForUpdates(
            forcePrimary, forceFallback,
            customOwner, customRepo, customBranch, customPath, customPat
        );
    });
}