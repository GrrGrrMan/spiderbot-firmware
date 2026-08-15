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
#include "CameraServer.h"

bool g_logEnabled = true;

NetworkManager netManager;
MQTTManager mqttManager;
ServoManager servoManager;
MotionController motionController(servoManager);
CommandDispatcher cmdDispatcher;

volatile unsigned long g_lastCmdTime = 0;


void TaskNetwork(void *pvParameters);
void TaskControl(void *pvParameters);
void TaskCameraStream(void *pvParameters);

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    g_logSink.begin(25);
    LOG_SYS("Booting esp-cam-main with Onboard Kinematics Engine...");

    otaManager.begin();

    // Register handlers passing the motion controller reference
    registerAllCommandHandlers(cmdDispatcher, servoManager, otaManager, motionController, mqttManager);

    mqttManager.setCommandCallback([](const String& type, JsonDocument& doc) {
        g_lastCmdTime = millis(); // Reset watchdog on ANY incoming command
        cmdDispatcher.dispatch(type, doc);
    });

    xTaskCreatePinnedToCore(TaskNetwork, "NetTask", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(TaskControl, "ControlTask", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(TaskCameraStream, "CamTask", 4096, NULL, 1, NULL, 0);
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
                mqttManager.sendConfig();
            }

            // Drain 1 log entry per network cycle to prevent TCP socket saturation
            LogEntry entry;
            if (g_logSink.pop(entry)) {
                mqttManager.sendLog(entry.message);
            }

            // Publish telemetry snapshot
            JsonDocument telemetry;
            telemetry["uptime"]    = millis() / 1000;
            telemetry["free_heap"] = ESP.getFreeHeap();
            telemetry["rssi"]      = WiFi.RSSI();
            telemetry["ip"]        = netManager.getLocalIP();
            telemetry["hotspot"]   = netManager.isHotspot();
            telemetry["power"]     = servoManager.isOutputsEnabled();

            mqttManager.sendTelemetry(telemetry);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void TaskControl(void *pvParameters) {
    servoManager.begin();
    motionController.begin();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); 

    for (;;) {
        // --- SAFETY WATCHDOG ---
        // If we haven't received an MQTT command in 2.0 seconds, halt everything.
        if (g_lastCmdTime > 0 && (millis() - g_lastCmdTime > 2000)) {
            VelocityCommand stopCmd = {0.0f, 0.0f, 0.0f, 25.0f, 1.0f, 0.0f, 0.0f};
            motionController.setVelocity(stopCmd);     // Stop walking
            servoManager.setOutputsEnabled(false);     // Cut PWM signals (go limp)
            g_lastCmdTime = 0;                         // Reset tracker to avoid log spam
            LOG_ERR("Watchdog Timeout! Connection lost. Halting motion and disabling servos.");
        }

        motionController.update(0.01f); 
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ── P2 Camera MJPEG stream ───────────────────────────────────────────────────
// Lives on core 0 (network core) so the 100 Hz control loop on core 1 is never
// starved. The httpd server only captures frames while a browser is connected,
// so an idle CAM costs ~nothing. Waits for WiFi/configuration to settle so the
// config payload reports a real IP in mjpeg_url.
void TaskCameraStream(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(2500));
    cameraServer.begin();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}