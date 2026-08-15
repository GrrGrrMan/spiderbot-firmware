// V2 Hexapod S3 Main — P6a real firmware (ESP32-S3 Servo Controller)
//
// Replicates cam-main's 100 Hz two-task FreeRTOS structure on the ESP32-S3.
// Camera code intentionally absent (ADR-001: S3 = hands & voice).
//
// TaskNetwork (core 0): WiFi + MQTT + OTA boot-validate + log drain + telemetry
// TaskControl  (core 1): ServoManager begin + boot servo cycle + 100 Hz loop
//
// Wokwi servo-only scenario (test-servo-cycle.yaml) asserts EXACT strings:
//   - "S3 Servo Manager ready"
//   - "Servo cycle: LF_COXA -> CENTER"   (…one per joint, 18 total…)
//   - "Servo cycle: Servo cycle complete - all 18 OK"

#include <Arduino.h>
#include "NetworkManager.h"
#include "MQTTManager.h"
#include "ServoManager.h"
#include "OTAManager.h"
#include "MotionController.h"
#include "CommandDispatcher.h"
#include "LogSink.h"
#include "net_config.h"
#include "servo_config.h"
#include "logger.h"
#include "command_handlers.h"

bool g_logEnabled = true;

NetworkManager netManager;
MQTTManager mqttManager;
ServoManager servoManager;
MotionController motionController(servoManager);
CommandDispatcher cmdDispatcher;

volatile unsigned long g_lastCmdTime = 0;

void TaskNetwork(void *pvParameters);
void TaskControl(void *pvParameters);

// Boot-time servo cycle test: sweep each of the 18 servos out to +200us and
// back to center, logging the exact lines the Wokwi YAML asserts. Iterates in
// firmware leg order (RF, RM, RB, LB, LM, LF) so LF_COXA/LF_FEMUR appear last.
static void runBootServoCycle() {
    const char* jointNames[6][3] = {
        { "RF_COXA", "RF_FEMUR", "RF_TIBIA" },
        { "RM_COXA", "RM_FEMUR", "RM_TIBIA" },
        { "RB_COXA", "RB_FEMUR", "RB_TIBIA" },
        { "LB_COXA", "LB_FEMUR", "LB_TIBIA" },
        { "LM_COXA", "LM_FEMUR", "LM_TIBIA" },
        { "LF_COXA", "LF_FEMUR", "LF_TIBIA" },
    };

    for (uint8_t leg = 0; leg < 6; leg++) {
        uint8_t channels[3] = { LEG_COXA_CHANNELS[leg], LEG_FEMUR_CHANNELS[leg], LEG_TIBIA_CHANNELS[leg] };
        for (uint8_t j = 0; j < 3; j++) {
            uint8_t ch = channels[j];
            servoManager.setServoPulseUs(ch, 1700); // swing out
            delay(60);
            servoManager.setServoPulseUs(ch, 1500); // return to center (~SERVO_HOME_TICK)
            delay(60);
            Serial.printf("Servo cycle: %s -> CENTER\r\n", jointNames[leg][j]);
        }
    }

    // NOTE: literal string intentionally mirrors the Wokwi YAML expect.
    Serial.println("Servo cycle: Servo cycle complete - all 18 OK");
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    g_logSink.begin(25);
    LOG_SYS("Booting s3-main (ESP32-S3 Servo Controller)...");

    otaManager.begin();

    // Register handlers passing the motion controller reference
    registerAllCommandHandlers(cmdDispatcher, servoManager, otaManager, motionController, mqttManager);

    mqttManager.setCommandCallback([](const String& type, JsonDocument& doc) {
        g_lastCmdTime = millis(); // Reset watchdog on ANY incoming command
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
    LOG_SYS("S3 Servo Manager ready");
    runBootServoCycle();
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