#include <Arduino.h>
#include "NetworkManager.h"
#include "MQTTManager.h"
#include "ServoManager.h"
#include "OTAManager.h"
#include "CommandDispatcher.h"
#include "LogSink.h"
#include "cmd_schema.h"
#include "net_config.h"
#include "servo_config.h"
#include "logger.h"


bool g_logEnabled = true;


NetworkManager netManager;
MQTTManager mqttManager;
ServoManager servoManager;
CommandDispatcher cmdDispatcher;

void TaskNetwork(void *pvParameters);
void TaskControl(void *pvParameters);
void setupCommandHandlers();

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    g_logSink.begin(25); // Initialize FreeRTOS log queue
    LOG_SYS("Booting esp-cam-main...");

    otaManager.begin();

    setupCommandHandlers();
    mqttManager.setCommandCallback([](const String& type, JsonDocument& doc) {
        cmdDispatcher.dispatch(type, doc);
    });

    xTaskCreatePinnedToCore(TaskNetwork, "NetTask", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(TaskControl, "ControlTask", 4096, NULL, 2, NULL, 1);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void setupCommandHandlers() {
    // 1. Single Servo Control Handler (Direct channel & microsecond pulse width)
    cmdDispatcher.registerHandler(CMD_TYPE_SERVO, [](const JsonDocument& doc) {
        uint8_t ch = doc["channel"] | 0;
        uint16_t pulseUs = doc["pulse_us"] | 1500;
        uint16_t offTick = (pulseUs * 4096UL) / 20000UL;
        servoManager.setPWM(ch, ch * STAGGER_OFFSET, offTick);
        LOG_MOT("Direct Servo Write: Ch %d -> %d us (%d ticks)", ch, pulseUs, offTick);
    });

    // 2. Batch Servo Control Handler
    cmdDispatcher.registerHandler(CMD_TYPE_SERVO_BATCH, [](const JsonDocument& doc) {
        JsonArrayConst servos = doc["servos"].as<JsonArrayConst>();
        for (JsonObjectConst s : servos) {
            uint8_t ch = s["ch"] | 0;
            uint16_t pulseUs = s["pulse_us"] | 1500;
            uint16_t offTick = (pulseUs * 4096UL) / 20000UL;
            servoManager.setPWM(ch, ch * STAGGER_OFFSET, offTick);
        }
        LOG_MOT("Batch Servo Write executed for %d servos", servos.size());
    });

    // 3. Motion Target Handler (Kinematics)
    cmdDispatcher.registerHandler(CMD_TYPE_MOTION, [](const JsonDocument& doc) {
        float vx = doc["vx"] | 0.0f;
        float vy = doc["vy"] | 0.0f;
        float yaw = doc["yaw"] | 0.0f;
        LOG_MOT("Target Velocity: vx=%.2f, vy=%.2f, yaw=%.2f", vx, vy, yaw);
    });

    // 4. System Control Handler
    cmdDispatcher.registerHandler(CMD_TYPE_SYSTEM, [](const JsonDocument& doc) {
        if (doc["logging"].is<bool>()) {
            g_logEnabled = doc["logging"].as<bool>();
            LOG_SYS("Logging state changed to: %d", g_logEnabled);
        }
    });
    // 5. OTA
    cmdDispatcher.registerHandler("ota", [](const JsonDocument& doc) {
        bool forceFallback = doc["fallback"] | false;
        bool forcePrimary  = doc["primary"]  | false;
        LOG_SYS("Remote OTA command received via MQTT!");
        otaManager.checkForUpdates(forcePrimary, forceFallback);
    });
}

void TaskNetwork(void *pvParameters) {
    netManager.begin();
    mqttManager.begin(DEVICE_ID, 1883);

    // Declare the boot validation flag here
    static bool s_bootValidated = false;

    for (;;) {
        netManager.update();
        bool netConnected = netManager.isConnected();
        const char* brokerHost = netManager.getMQTTBroker();

        mqttManager.update(netConnected, brokerHost);

        if (netConnected && mqttManager.isConnected()) {
            // Validate boot image ONCE after network confirmation to cancel partition rollback
            if (!s_bootValidated) {
                s_bootValidated = true;
                otaManager.validateBootImage();
            }

            // 1. Drain log queue and publish to hexapod/hexapod-cam-01/logs
            LogEntry entry;
            while (g_logSink.pop(entry)) {
                mqttManager.sendLog(entry.message);
            }

            // 2. Enrich telemetry snapshot and publish to hexapod/hexapod-cam-01/telemetry
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

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}