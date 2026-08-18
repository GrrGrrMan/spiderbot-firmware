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
#include "ServoManager.h"
#include "MotionController.h"

bool g_logEnabled = true;

NetworkManager netManager;
MQTTManager mqttManager;
ServoManager servoManager;
MotionController motionController(servoManager);
CommandDispatcher cmdDispatcher;

#ifdef CAM_ENABLE_SERVO
volatile unsigned long g_lastCmdTime = 0;
void TaskControl(void *pvParameters);
#endif

void TaskNetwork(void *pvParameters);

void setup() {
    Serial.begin(115200);
    delay(1000);

    g_logSink.begin(25);
#ifdef CAM_ENABLE_SERVO
    LOG_SYS("Booting esp-cam-main with Onboard Kinematics Engine...");
#else
    LOG_SYS("Booting esp-cam-main in CAMERA-ONLY mode (eyes only, no motion)...");
#endif

    otaManager.begin();

    // Register command handlers
    registerAllCommandHandlers(cmdDispatcher, servoManager, otaManager, motionController, mqttManager);

    mqttManager.setCommandCallback([](const String& type, JsonDocument& doc) {
#ifdef CAM_ENABLE_SERVO
        g_lastCmdTime = millis();
#endif
        cmdDispatcher.dispatch(type, doc);
    });

    // Core 0: Dedicated to Network, MQTT, and Logging
    xTaskCreatePinnedToCore(TaskNetwork, "NetTask", 8192, NULL, 1, NULL, 0);

#ifdef CAM_ENABLE_SERVO
    // Core 1: 100 Hz kinematic control loop (if servos enabled)
    xTaskCreatePinnedToCore(TaskControl, "ControlTask", 4096, NULL, 2, NULL, 1);
#endif
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void TaskNetwork(void *pvParameters) {
    netManager.begin();
    mqttManager.begin(DEVICE_ID, 1883);

    static bool s_bootValidated = false;
    static bool s_cameraAttempted = false;
    static uint8_t s_cameraRetries = 0;
    const uint8_t MAX_CAM_RETRIES = 3;

    for (;;) {
        netManager.update();
        bool netConnected = netManager.isConnected();
        const char* brokerHost = netManager.getMQTTBroker();

        mqttManager.update(netConnected, brokerHost);

        if (netConnected) {
            // Attempt camera init up to MAX_CAM_RETRIES times
            if (!s_cameraAttempted && s_cameraRetries < MAX_CAM_RETRIES) {
                if (cameraServer.begin()) {
                    s_cameraAttempted = true; // Successfully started
                } else {
                    s_cameraRetries++;
                    if (s_cameraRetries >= MAX_CAM_RETRIES) {
                        s_cameraAttempted = true; // Give up and stop spamming
                        LOG_ERR("CAM: Max camera init retries reached. Camera disabled.");
                    }
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

                // Publish rich telemetry snapshot with dynamic MJPEG URL
                JsonDocument telemetry;
                telemetry["uptime"]     = millis() / 1000;
                telemetry["free_heap"]  = ESP.getFreeHeap();
                telemetry["rssi"]       = WiFi.RSSI();
                telemetry["ip"]         = netManager.getLocalIP();
                telemetry["hotspot"]    = netManager.isHotspot();
                telemetry["stream_url"] = "http://" + netManager.getLocalIP() + ":" + String(CAM_STREAM_PORT) + "/stream";
#ifdef CAM_ENABLE_SERVO
                telemetry["power"]      = servoManager.isOutputsEnabled();
#endif
                mqttManager.sendTelemetry(telemetry);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

#ifdef CAM_ENABLE_SERVO
void TaskControl(void *pvParameters) {
    servoManager.begin();
    motionController.begin();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); 

    for (;;) {
        // --- SAFETY WATCHDOG ---
        if (g_lastCmdTime > 0 && (millis() - g_lastCmdTime > 2000)) {
            VelocityCommand stopCmd = {0.0f, 0.0f, 0.0f, 25.0f, 1.0f, 0.0f, 0.0f};
            motionController.setVelocity(stopCmd);     // Stop walking
            servoManager.setOutputsEnabled(false);     // Cut PWM signals (go limp)
            g_lastCmdTime = 0;
            LOG_ERR("Watchdog Timeout! Connection lost. Halting motion and disabling servos.");
        }

        motionController.update(0.01f);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
#endif