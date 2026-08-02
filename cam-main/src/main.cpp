#include <Arduino.h>
#include "NetworkManager.h"
#include "MQTTManager.h"
#include "ServoManager.h"
#include "OTAManager.h"
#include "MotionController.h"
#include "CommandDispatcher.h"
#include "LogSink.h"
#include "net_config.h"
#include "logger.h"
#include "command_handlers.h"

bool g_logEnabled = true;

NetworkManager netManager;
MQTTManager mqttManager;
ServoManager servoManager;
MotionController motionController(servoManager);
CommandDispatcher cmdDispatcher;

void TaskNetwork(void *pvParameters);
void TaskControl(void *pvParameters);

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    g_logSink.begin(25);
    LOG_SYS("Booting esp-cam-main with Onboard Kinematics Engine...");

    otaManager.begin();

    // Register handlers passing the motion controller reference
    registerAllCommandHandlers(cmdDispatcher, servoManager, otaManager, motionController);

    mqttManager.setCommandCallback([](const String& type, JsonDocument& doc) {
        cmdDispatcher.dispatch(type, doc);
    });

    xTaskCreatePinnedToCore(TaskNetwork, "NetTask", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(TaskControl, "ControlTask", 4096, NULL, 2, NULL, 1);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void TaskNetwork(void *pvParameters) {
    netManager.begin();
    mqttManager.begin(DEVICE_ID, 1883);

    static bool s_bootValidated = false;

    for (;;) {
        netManager.update();
        bool netConnected = netManager.isConnected();
        const char* brokerHost = netManager.getMQTTBroker();

        mqttManager.update(netConnected, brokerHost);

        if (netConnected && mqttManager.isConnected()) {
            if (!s_bootValidated) {
                s_bootValidated = true;
                otaManager.validateBootImage();
            }

            // Drain log queue to MQTT
            LogEntry entry;
            while (g_logSink.pop(entry)) {
                mqttManager.sendLog(entry.message);
            }

            // Publish telemetry snapshot
            JsonDocument telemetry;
            telemetry["uptime"]    = millis() / 1000;
            telemetry["free_heap"] = ESP.getFreeHeap();
            telemetry["rssi"]      = WiFi.RSSI();
            telemetry["ip"]        = netManager.getLocalIP();
            telemetry["hotspot"]   = netManager.isHotspot();

            mqttManager.sendTelemetry(telemetry);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void TaskControl(void *pvParameters) {
    servoManager.begin();
    motionController.begin();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz loop (10ms)

    for (;;) {
        motionController.update(0.01f); // Update motion engine with 10ms time delta
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}