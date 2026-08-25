// V2 Hexapod S3 Main — ESP32-S3 Servo Controller & Streaming Audio Node
//
// Dual-Core FreeRTOS Architecture:
//   TaskNetwork (Core 0): Wi-Fi + MQTT + Log Sink + Telemetry + Base64 Stream Decoding
//   TaskControl (Core 1): 100 Hz Kinematics Loop + Two-Stage Watchdog + PCA9685 Servos
//   TaskAudio   (Core 1): I2S Audio DMA Consumer (RingBuffer Stream + Tones + Alarms)

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"

#include "NetworkManager.h"
#include "MQTTManager.h"
#include "ServoManager.h"
#include "OTAManager.h"
#include "MotionController.h"
#include "CommandDispatcher.h"
#include "LogSink.h"
#include "net_config.h"
#include "servo_config.h"
#include "audio_config.h"
#include "motion_config.h"
#include "logger.h"
#include "command_handlers.h"
#include "AudioManager.h"
#include "TTSStreamer.h"
#include "esp_heap_caps.h"

bool g_logEnabled = true;

NetworkManager   netManager;
MQTTManager      mqttManager;
ServoManager     servoManager;
MotionController motionController(servoManager);
CommandDispatcher cmdDispatcher;
AudioManager     audioManager;
TTSStreamer      ttsStreamer;

volatile unsigned long g_lastActivityTime = 0;
volatile unsigned long g_lastMotionCmdTime = 0;

// Audio command tokens sent from TaskNetwork (Core 0) to TaskAudio (Core 1)
enum class AudioCommandType : uint8_t {
    TONE       = 0,
    ALARM      = 1,
    TTS_START  = 2,
    TTS_END    = 3
};

struct AudioCommand {
    AudioCommandType type;
    uint16_t         freqHz;
    uint16_t         ms;
    char             alarmName[16];
};

static QueueHandle_t     g_audioQueue       = nullptr;
static RingbufHandle_t   g_pcmRingBuffer    = nullptr;
static volatile bool     g_audioDonePending = false;
volatile bool g_watchdogBraked = false;
static char              g_audioIdleAction[16] = {0};

void TaskNetwork(void *pvParameters);
void TaskControl(void *pvParameters);
void TaskAudio(void *pvParameters);

static inline bool isAudioBusy() {
    return (audioManager.state() == AudioState::PLAYING) || 
           ttsStreamer.isActive();
}

void setup() {
    pinMode(PIN_PCA_OE, OUTPUT);
    digitalWrite(PIN_PCA_OE, HIGH); // Assert OE high immediately on reset
    Serial.begin(115200);
    delay(500);

    g_logSink.begin(25);
    LOG_SYS("Booting s3-main (ESP32-S3 Servo & Streaming Audio Node)...");

    otaManager.begin();

    // 512KB RingBuffer (Struct control block in Internal SRAM, Buffer data in PSRAM)
    if (psramFound()) {
        StaticRingbuffer_t *rb_struct = (StaticRingbuffer_t *)heap_caps_malloc(sizeof(StaticRingbuffer_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        uint8_t *rb_storage = (uint8_t *)heap_caps_malloc(512 * 1024, MALLOC_CAP_SPIRAM);

        if (rb_struct && rb_storage) {
            g_pcmRingBuffer = xRingbufferCreateStatic(512 * 1024, RINGBUF_TYPE_BYTEBUF, rb_storage, rb_struct);
            LOG_SYS("Allocated 512KB Audio RingBuffer in PSRAM (holds ~12s speech)");
        } else {
            LOG_ERR("PSRAM allocation failed, falling back to SRAM");
        }
    }

    if (!g_pcmRingBuffer) {
        g_pcmRingBuffer = xRingbufferCreate(48 * 1024, RINGBUF_TYPE_BYTEBUF);
        LOG_SYS("Allocated 48KB Internal SRAM RingBuffer fallback");
    }

    g_audioQueue = xQueueCreate(16, sizeof(AudioCommand));
    audioManager.begin();
    ttsStreamer.begin();

    registerAllCommandHandlers(cmdDispatcher, servoManager, otaManager, motionController, mqttManager);

    mqttManager.setCommandCallback([](const String& type, JsonDocument& doc) {
        unsigned long now = millis();
        
        // Heartbeats must NOT reset the user inactivity (auto-limp) timer
        if (type != "heartbeat") {
            g_lastActivityTime = now;
        }
        
        if (type == "motion" || type == "keyframe" || type == "pose" || type == "sequence" || type == "preset") {
            g_lastMotionCmdTime = now;
            g_watchdogBraked = false; // Reset flag when user sends a new command
        }
        cmdDispatcher.dispatch(type, doc);
    });

    mqttManager.setAudioCommandCallback([](const String& action, JsonDocument& doc) {
        g_lastActivityTime = millis();

        // 1. Master Volume Tuning
        if (action == "volume" || doc["volume"].is<float>()) {
            float v = doc["volume"] | audioManager.getVolume();
            audioManager.setVolume(v);
            LOG_SYS("AUDIO: Master volume set to %.0f%%", v * 100.0f);
            return;
        }

        // 2. Sound Effects & Alarms
        AudioCommand cmd{};
        if (action == "beep" || action == "play") {
            cmd.type = AudioCommandType::TONE;
            cmd.freqHz = (action == "play") ? 660 : 1200;
            cmd.ms = 100;
            mqttManager.sendAudioStatus("playing", action.c_str());
            if (g_audioQueue) xQueueSend(g_audioQueue, &cmd, 0);
        } else if (action == "alarm") {
            const char* name = doc["payload"] | "idle";
            cmd.type = AudioCommandType::ALARM;
            strncpy(cmd.alarmName, name, sizeof(cmd.alarmName) - 1);
            cmd.alarmName[sizeof(cmd.alarmName) - 1] = '\0';
            mqttManager.sendAudioStatus("playing", "alarm");
            if (g_audioQueue) xQueueSend(g_audioQueue, &cmd, 0);
        }
    });

    // ── Ultra-Low Latency Binary Audio Ingestion (Core 0) ──
    mqttManager.setAudioBinCommandCallback([](const uint8_t* payload, size_t length) {
        g_lastActivityTime = millis();
        if (length < 10) return; // Must contain at least the 10-byte header
        
        uint8_t action = payload[1];

        // 0x00 = TTS CHUNK
        if (action == 0x00) {
            uint32_t flowId;
            uint16_t seq, total;
            
            // Unpack header using memory-safe extraction
            memcpy(&flowId, payload + 2, 4);
            memcpy(&seq, payload + 6, 2);
            memcpy(&total, payload + 8, 2);

            const uint8_t* pcmData = payload + 10;
            size_t pcmLen = length - 10;

            if (seq == 0) {
                mqttManager.sendAudioStatus("playing", "tts");
                AudioCommand startCmd{};
                startCmd.type = AudioCommandType::TTS_START;
                if (g_audioQueue) xQueueSend(g_audioQueue, &startCmd, 0);
            }

            int16_t* pcmChunk = nullptr;
            size_t samples = 0;

            TTSStreamer::FeedResult res = ttsStreamer.feedBinary(flowId, seq, total, pcmData, pcmLen, &pcmChunk, &samples);

            if (samples > 0 && pcmChunk != nullptr && g_pcmRingBuffer) {
                // Instantly ship to Core 1 DMA
                BaseType_t ok = xRingbufferSend(g_pcmRingBuffer, pcmChunk, samples * sizeof(int16_t), pdMS_TO_TICKS(50));
                if (ok != pdTRUE) {
                    LOG_ERR("AUDIO: RingBuffer full! Frame %u dropped.", seq);
                }
            }

            if (res == TTSStreamer::FeedResult::FLOW_COMPLETE) {
                ttsStreamer.resetFlow();
                AudioCommand endCmd{};
                endCmd.type = AudioCommandType::TTS_END;
                if (g_audioQueue) xQueueSend(g_audioQueue, &endCmd, pdMS_TO_TICKS(20));
            } else if (res == TTSStreamer::FeedResult::ERROR) {
                ttsStreamer.resetFlow();
                mqttManager.sendAudioStatus("error", "tts");
            }
        }
    });

    xTaskCreatePinnedToCore(TaskNetwork, "NetTask",     8192, NULL, 2, NULL, 0); // Core 0: Wi-Fi & MQTT (Priority 2)
    xTaskCreatePinnedToCore(TaskAudio,   "AudioTask",   8192, NULL, 1, NULL, 0); // Core 0: Audio DMA Streamer (Priority 1)
    xTaskCreatePinnedToCore(TaskControl, "ControlTask", 4096, NULL, 3, NULL, 1); // Core 1: 100% Dedicated Motion (Priority 3)
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void TaskNetwork(void *pvParameters) {
    netManager.begin();
    mqttManager.begin(DEVICE_ID, 1883);

    static unsigned long s_lastTelemetryMs = 0;
    static bool s_bootValidated = false;

    for (;;) {
        netManager.update();
        bool netConnected = netManager.isConnected();
        const char* brokerHost = netManager.getMQTTBroker();

        if (netConnected && brokerHost) {
            mqttManager.update(netConnected, brokerHost);
        }

        // Asynchronous non-blocking log drainer on Core 0 (Serial + MQTT)
        LogEntry entry;
        while (g_logSink.pop(entry)) {
            Serial.println(entry.message);
            if (netConnected && mqttManager.isConnected()) {
                mqttManager.sendLog(entry.message);
            }
        }

        if (netConnected && mqttManager.isConnected()) {
            if (g_audioDonePending) {
                g_audioDonePending = false;
                mqttManager.sendAudioStatus("idle", g_audioIdleAction[0] ? g_audioIdleAction : "tts");
            }

            unsigned long now = millis();
            if (now - s_lastTelemetryMs >= 100) {
                s_lastTelemetryMs = now;

                if (!s_bootValidated) {
                    s_bootValidated = true;
                    otaManager.validateBootImage();
                    mqttManager.sendConfig();

                    // ── Cheerful Startup Chime (Boot Confirmation Beep) ──
                    AudioCommand bootBeep{};
                    bootBeep.type = AudioCommandType::ALARM;
                    strncpy(bootBeep.alarmName, "curious", sizeof(bootBeep.alarmName) - 1);
                    if (g_audioQueue) xQueueSend(g_audioQueue, &bootBeep, 0);
                }

                JsonDocument telemetry;
                telemetry["uptime"]          = now / 1000;
                telemetry["free_heap"]       = ESP.getFreeHeap();
                telemetry["rssi"]            = WiFi.RSSI();
                telemetry["ip"]              = netManager.getLocalIP();
                telemetry["hotspot"]         = netManager.isHotspot();
                telemetry["power"]           = servoManager.isOutputsEnabled();
                telemetry["audio"]           = isAudioBusy() ? "playing" : "idle";
                telemetry["watchdog_braked"] = g_watchdogBraked;
                
                mqttManager.sendTelemetry(telemetry);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void TaskControl(void *pvParameters) {
    servoManager.begin();
    LOG_SYS("S3 Servo Manager ready");

    motionController.begin();

    // Initialize activity timer at boot
    g_lastActivityTime = millis();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    
    unsigned long activeMotionStartMs = 0;

    for (;;) {
        unsigned long now = millis();
        bool isMoving = motionController.isMoving();
        bool audioActive = isAudioBusy();

        // Audio or motion keeps the overall system awake (prevents OE limp sleep)
        if (isMoving || audioActive) {
            g_lastActivityTime = now;
        }

        // Active locomotion or sequence playback keeps the gait motion timer refreshed
        if (isMoving) {
            g_lastMotionCmdTime = now;
        }

        // ── WATCHDOG STAGE 1: Velocity Auto-Brake (3s) ──
        if (g_lastMotionCmdTime > 0) {
            if (activeMotionStartMs == 0) activeMotionStartMs = now;

            if (MAX_CONTINUOUS_MOTION_MS > 0 && (now - activeMotionStartMs > MAX_CONTINUOUS_MOTION_MS)) {
                VelocityCommand stopCmd = {0.0f, 0.0f, 0.0f, 25.0f, 1.0f, 0.0f, 0.0f};
                motionController.setVelocity(stopCmd);
                motionController.stopSequence();
                activeMotionStartMs = 0;
                g_lastMotionCmdTime = 0;
                LOG_ERR("Safety Cap: Continuous motion exceeded %ums -> Holding stance.", MAX_CONTINUOUS_MOTION_MS);
            }
            else if (MOTION_WATCHDOG_TIMEOUT_MS > 0 && (now - g_lastMotionCmdTime > MOTION_WATCHDOG_TIMEOUT_MS)) {
                VelocityCommand stopCmd = {0.0f, 0.0f, 0.0f, 25.0f, 1.0f, 0.0f, 0.0f};
                motionController.setVelocity(stopCmd);
                activeMotionStartMs = 0;
                g_lastMotionCmdTime = 0;
                g_watchdogBraked = true; // Tell the UI we hit the brakes!
                LOG_MOT("Motion Watchdog: Velocity timed out -> holding stance.");
            }
        } else {
            activeMotionStartMs = 0;
        }

        // ── WATCHDOG STAGE 2: Deep Limp Sleep (OE pulled HIGH after 15s idle) ──
        if (servoManager.isOutputsEnabled() && (now - g_lastActivityTime > INACTIVITY_SLEEP_TIMEOUT_MS)) {
            servoManager.setOutputsEnabled(false); // Powers off PCA9685 PWM & asserts OE HIGH
            LOG_SYS("Inactivity Watchdog: %ums idle -> Servos powered down (OE HIGH / LIMP).", INACTIVITY_SLEEP_TIMEOUT_MS);
        }

        motionController.update(0.01f);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskAudio(void *pvParameters) {
    AudioCommand cmd;
    bool isStreamingTts = false;
    bool isPrebuffered  = false;
    bool isTtsEnding    = false;
    unsigned long lastAudioDataMs = 0;

    const size_t LOW_LATENCY_PREBUFFER = 16384; 

    for (;;) {
        TickType_t waitTicks = isStreamingTts ? pdMS_TO_TICKS(2) : portMAX_DELAY;

        if (xQueueReceive(g_audioQueue, &cmd, waitTicks) == pdTRUE) {
            switch (cmd.type) {
                case AudioCommandType::TONE:
                    audioManager.playTone(cmd.freqHz, cmd.ms);
                    strncpy(g_audioIdleAction, "tone", sizeof(g_audioIdleAction) - 1);
                    g_audioDonePending = true;
                    break;

                case AudioCommandType::ALARM:
                    audioManager.playAlarm(cmd.alarmName[0] ? cmd.alarmName : "idle");
                    strncpy(g_audioIdleAction, "alarm", sizeof(g_audioIdleAction) - 1);
                    g_audioDonePending = true;
                    break;

                case AudioCommandType::TTS_START:
                    isStreamingTts = true;
                    isPrebuffered  = false;
                    isTtsEnding    = false;
                    lastAudioDataMs = millis();
                    break;

                case AudioCommandType::TTS_END:
                    isTtsEnding = true;
                    break;
            }
        }

        // Real-time Core 0 I2S DMA Streamer
        if (isStreamingTts && g_pcmRingBuffer) {
            size_t waitingBytes = 0;
            vRingbufferGetInfo(g_pcmRingBuffer, nullptr, nullptr, nullptr, nullptr, (UBaseType_t*)&waitingBytes);

            if (waitingBytes > 0) {
                lastAudioDataMs = millis();
            }

            if (!isPrebuffered) {
                if (waitingBytes >= LOW_LATENCY_PREBUFFER || isTtsEnding) {
                    isPrebuffered = true;
                } else if (millis() - lastAudioDataMs > 1500) {
                    // Prebuffer timeout for very short utterances
                    isPrebuffered = true;
                } else {
                    vTaskDelay(pdMS_TO_TICKS(2));
                    continue;
                }
            }

            size_t itemSize = 0;
            void* item = xRingbufferReceiveUpTo(g_pcmRingBuffer, &itemSize, pdMS_TO_TICKS(2), 4096);
            if (item && itemSize > 0) {
                audioManager.writePcmChunk((const int16_t*)item, itemSize / sizeof(int16_t));
                vRingbufferReturnItem(g_pcmRingBuffer, item);
                lastAudioDataMs = millis();
            } else if (waitingBytes == 0) {
                if (isTtsEnding || (millis() - lastAudioDataMs > 2000)) {
                    audioManager.stop();
                    ttsStreamer.resetFlow();
                    isStreamingTts = false;
                    isPrebuffered  = false;
                    isTtsEnding    = false;
                    strncpy(g_audioIdleAction, "tts", sizeof(g_audioIdleAction) - 1);
                    g_audioDonePending = true;
                    LOG_SYS("AUDIO TTS playback complete");
                }
                vTaskDelay(pdMS_TO_TICKS(2));
            }
        }
    }
}