#include <Arduino.h>
#include "NetworkManager.h"
#include "MQTTManager.h"
#include "OTAManager.h"
#include "CommandDispatcher.h"
#include "LogSink.h"
#include "net_config.h"
#include "logger.h"
#include "command_handlers.h"
#include "CameraServer.h"

bool g_logEnabled = true;

NetworkManager    netManager;
MQTTManager       mqttManager;
CommandDispatcher cmdDispatcher;

void TaskNetwork(void *pvParameters);

void setup() {
    Serial.begin(115200);
    delay(1000);

    g_logSink.begin(25);
    LOG_SYS("Booting esp-cam-main in CAMERA-ONLY mode (eyes only)...");

    otaManager.begin();

    // Pass cameraServer into command handlers for remote agent tuning
    registerAllCommandHandlers(cmdDispatcher, otaManager, mqttManager, cameraServer);

    mqttManager.setCommandCallback([](const String& type, JsonDocument& doc) {
        cmdDispatcher.dispatch(type, doc);
    });

    // Core 0: Wi-Fi, MQTT, Telemetry, Logging, OTA
    // Core 1: CameraServer HTTP MJPEG streaming (:81/stream)
    xTaskCreatePinnedToCore(TaskNetwork, "NetTask", 8192, NULL, 1, NULL, 0);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void TaskNetwork(void *pvParameters) {
    netManager.begin();
    mqttManager.begin(DEVICE_ID, 1883);

    static bool s_bootValidated = false;
    static unsigned long s_lastCamAttemptMs = 0;
    static unsigned long s_lastTelemetryMs = 0;

    for (;;) {
        netManager.update();
        bool netConnected = netManager.isConnected();
        const char* brokerHost = netManager.getMQTTBroker();

        mqttManager.update(netConnected, brokerHost);

        if (netConnected) {
            // Continuously retry camera init every 4 seconds until online
            if (!cameraServer.isRunning()) {
                unsigned long now = millis();
                if (now - s_lastCamAttemptMs >= 4000UL) {
                    s_lastCamAttemptMs = now;
                    LOG_NET("CAM: Initializing camera driver...");
                    cameraServer.begin();
                }
            }

            if (mqttManager.isConnected()) {
                if (!s_bootValidated) {
                    s_bootValidated = true;
                    otaManager.validateBootImage();
                    mqttManager.sendConfig();
                }

                // Drain 1 log entry per network cycle
                LogEntry entry;
                if (g_logSink.pop(entry)) {
                    mqttManager.sendLog(entry.message);
                }

                // Publish telemetry once per second (1000ms)
                unsigned long now = millis();
                if (now - s_lastTelemetryMs >= 1000UL) {
                    s_lastTelemetryMs = now;

                    JsonDocument telemetry;
                    telemetry["uptime"]     = now / 1000;
                    telemetry["free_heap"]  = ESP.getFreeHeap();
                    telemetry["rssi"]       = WiFi.RSSI();
                    telemetry["ip"]         = netManager.getLocalIP();
                    telemetry["hotspot"]    = netManager.isHotspot();
                    telemetry["stream_url"] = "http://" + netManager.getLocalIP() + ":" + String(CAM_STREAM_PORT) + "/stream";
                    telemetry["flash_pct"]  = cameraServer.getFlashlight();
                    telemetry["target_fps"] = cameraServer.getTargetFps();
                    mqttManager.sendTelemetry(telemetry);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}