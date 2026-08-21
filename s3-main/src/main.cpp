// V2 Hexapod S3 Main — ESP32-S3 Servo Controller & Streaming Audio Node
//
// Dual-Core FreeRTOS Architecture:
//   TaskNetwork (Core 0): Wi-Fi + MQTT + Log Sink + Telemetry + Base64 Stream Decoding
//   TaskControl (Core 1): 100 Hz Kinematics Loop + Safety Watchdog + PCA9685 Servos
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
#include "logger.h"
#include "command_handlers.h"
#include "AudioManager.h"
#include "audio_config.h"
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

volatile unsigned long g_lastCmdTime = 0;

// Audio command tokens sent from TaskNetwork (Core 0) to TaskAudio (Core 1)
enum class AudioCommandType : uint8_t {
    TONE       = 0,
    ALARM      = 1,
    TTS_STREAM = 2,
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
static char              g_audioIdleAction[16] = {0};

void TaskNetwork(void *pvParameters);
void TaskControl(void *pvParameters);
void TaskAudio(void *pvParameters);

void setup() {
    Serial.begin(115200);
    delay(1000);

    g_logSink.begin(25);
    LOG_SYS("Booting s3-main (ESP32-S3 Servo & Streaming Audio Node)...");

    otaManager.begin();

    // 512KB RingBuffer in PSRAM (holds ~12s speech without frame drops)
    if (psramFound()) {
        StaticRingbuffer_t *rb_struct = (StaticRingbuffer_t *)heap_caps_malloc(sizeof(StaticRingbuffer_t), MALLOC_CAP_SPIRAM);
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

    g_audioQueue = xQueueCreate(8, sizeof(AudioCommand));
    ttsStreamer.begin();

    registerAllCommandHandlers(cmdDispatcher, servoManager, otaManager, motionController, mqttManager);

    mqttManager.setCommandCallback([](const String& type, JsonDocument& doc) {
        g_lastCmdTime = millis();
        cmdDispatcher.dispatch(type, doc);
    });

    // Dedicated MQTT audio receiver (Core 0 decodes Base64; zero I2S blocking)
    mqttManager.setAudioCommandCallback([](const String& action, JsonDocument& doc) {
        g_lastCmdTime = millis();

        if (action == "tts") {
            const char* flowId  = doc["flow_id"] | "";
            uint16_t    seq     = doc["seq"]     | 0;
            uint16_t    total   = doc["total"]   | 0;
            const char* payload = doc["payload"] | "";

            if (seq == 0) {
                mqttManager.sendAudioStatus("playing", "tts");
            }

            int16_t* pcmChunk = nullptr;
            size_t   samples  = 0;

            TTSStreamer::FeedResult res = ttsStreamer.feed(flowId, seq, total, payload, &pcmChunk, &samples);

            if (samples > 0 && pcmChunk != nullptr && g_pcmRingBuffer) {
                BaseType_t ok = xRingbufferSend(g_pcmRingBuffer, pcmChunk, samples * sizeof(int16_t), pdMS_TO_TICKS(200));
                if (ok != pdTRUE) {
                    LOG_ERR("AUDIO: RingBuffer full! Frame %u dropped.", seq);
                }
            }

            AudioCommand cmd{};
            if (res == TTSStreamer::FeedResult::FLOW_COMPLETE) {
                cmd.type = AudioCommandType::TTS_END;
                if (g_audioQueue) xQueueSend(g_audioQueue, &cmd, portMAX_DELAY);
            } else if (res == TTSStreamer::FeedResult::CHUNK_READY || res == TTSStreamer::FeedResult::OK) {
                cmd.type = AudioCommandType::TTS_STREAM;
                if (g_audioQueue) xQueueSend(g_audioQueue, &cmd, 0);
            } else if (res == TTSStreamer::FeedResult::ERROR) {
                ttsStreamer.resetFlow();
                mqttManager.sendAudioStatus("error", "tts");
            }
            return;
        }

        AudioCommand cmd{};
        if (action == "beep" || action == "play") {
            cmd.type = AudioCommandType::TONE;
            cmd.freqHz = (action == "play") ? 660 : 1200;
            cmd.ms = 120;
            mqttManager.sendAudioStatus("playing", action.c_str());
            if (g_audioQueue) xQueueSend(g_audioQueue, &cmd, 0);
        } else if (action == "alarm") {
            const char* name = doc["payload"] | "idle";
            cmd.type = AudioCommandType::ALARM;
            strncpy(cmd.alarmName, name, sizeof(cmd.alarmName) - 1);
            mqttManager.sendAudioStatus("playing", "alarm");
            if (g_audioQueue) xQueueSend(g_audioQueue, &cmd, 0);
        }
    });

    xTaskCreatePinnedToCore(TaskNetwork, "NetTask",     8192, NULL, 1, NULL, 0); // Core 0
    xTaskCreatePinnedToCore(TaskControl, "ControlTask", 4096, NULL, 2, NULL, 1); // Core 1
    xTaskCreatePinnedToCore(TaskAudio,   "AudioTask",   8192, NULL, 2, NULL, 1); // Core 1
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

        unsigned long now = millis();
        if (netConnected && mqttManager.isConnected() && (now - s_lastTelemetryMs >= 100)) {
            s_lastTelemetryMs = now;

            if (!s_bootValidated) {
                s_bootValidated = true;
                otaManager.validateBootImage();
                mqttManager.sendConfig();
            }

            LogEntry entry;
            if (g_logSink.pop(entry)) {
                mqttManager.sendLog(entry.message);
            }

            JsonDocument telemetry;
            telemetry["uptime"]    = now / 1000;
            telemetry["free_heap"] = ESP.getFreeHeap();
            telemetry["rssi"]      = WiFi.RSSI();
            telemetry["ip"]        = netManager.getLocalIP();
            telemetry["hotspot"]   = netManager.isHotspot();
            telemetry["power"]     = servoManager.isOutputsEnabled();
            mqttManager.sendTelemetry(telemetry);

            if (g_audioDonePending) {
                g_audioDonePending = false;
                mqttManager.sendAudioStatus("idle", g_audioIdleAction[0] ? g_audioIdleAction : "tts");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void TaskControl(void *pvParameters) {
    servoManager.begin();
    LOG_SYS("S3 Servo Manager ready");

    if (audioManager.begin()) {
        audioManager.playTone(440, 120);
        audioManager.playAlarm("idle");
    }

    motionController.begin();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    for (;;) {
        bool isAudioBusy = (audioManager.state() == AudioState::PLAYING);
        if (isAudioBusy) {
            g_lastCmdTime = millis();
        }

        if (g_lastCmdTime > 0 && (millis() - g_lastCmdTime > 3000)) {
            VelocityCommand stopCmd = {0.0f, 0.0f, 0.0f, 25.0f, 1.0f, 0.0f, 0.0f};
            motionController.setVelocity(stopCmd);
            servoManager.setOutputsEnabled(false);
            g_lastCmdTime = 0;
            LOG_ERR("Watchdog Timeout! Connection lost. Halting motion and disabling servos.");
        }

        motionController.update(0.01f);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskAudio(void *pvParameters) {
    AudioCommand cmd;
    bool isStreamingTts = false;
    bool isPrebuffered  = false;

    // 500ms safety cushion prevents stuttering
    const size_t INITIAL_PREBUFFER = 22050; 

    for (;;) {
        TickType_t waitTicks = isStreamingTts ? pdMS_TO_TICKS(2) : portMAX_DELAY;

        if (xQueueReceive(g_audioQueue, &cmd, waitTicks) == pdTRUE) {
            switch (cmd.type) {
                case AudioCommandType::TONE:
                    isStreamingTts = false;
                    audioManager.playTone(cmd.freqHz, cmd.ms);
                    strncpy(g_audioIdleAction, "tone", sizeof(g_audioIdleAction) - 1);
                    g_audioDonePending = true;
                    break;

                case AudioCommandType::ALARM:
                    isStreamingTts = false;
                    audioManager.playAlarm(cmd.alarmName[0] ? cmd.alarmName : "idle");
                    strncpy(g_audioIdleAction, "alarm", sizeof(g_audioIdleAction) - 1);
                    g_audioDonePending = true;
                    break;

                case AudioCommandType::TTS_STREAM:
                    isStreamingTts = true;
                    break;

                case AudioCommandType::TTS_END:
                    // Drain all remaining audio in PSRAM to I2S
                    if (g_pcmRingBuffer) {
                        size_t itemSize = 0;
                        while (true) {
                            void* item = xRingbufferReceiveUpTo(g_pcmRingBuffer, &itemSize, pdMS_TO_TICKS(50), 2048);
                            if (!item || itemSize == 0) break;
                            audioManager.writePcmChunk((const int16_t*)item, itemSize / sizeof(int16_t));
                            vRingbufferReturnItem(g_pcmRingBuffer, item);
                        }
                    }

                    // Wait 200ms for DMA to physically finish playing the last word
                    vTaskDelay(pdMS_TO_TICKS(200));

                    audioManager.stop();
                    isStreamingTts = false;
                    isPrebuffered  = false;
                    strncpy(g_audioIdleAction, "tts", sizeof(g_audioIdleAction) - 1);
                    g_audioDonePending = true;
                    LOG_SYS("AUDIO TTS playback complete");
                    break;
            }
        }

        // Real-time I2S DMA feeding
        if (isStreamingTts && g_pcmRingBuffer) {
            size_t waitingBytes = 0;
            vRingbufferGetInfo(g_pcmRingBuffer, nullptr, nullptr, nullptr, nullptr, &waitingBytes);

            // 1. Prebuffer gate
            if (!isPrebuffered) {
                if (waitingBytes >= INITIAL_PREBUFFER) {
                    isPrebuffered = true;
                } else {
                    vTaskDelay(pdMS_TO_TICKS(2));
                    continue;
                }
            }

            // 2. Continuous 2048-byte drain
            size_t itemSize = 0;
            void* item = xRingbufferReceiveUpTo(g_pcmRingBuffer, &itemSize, pdMS_TO_TICKS(2), 2048);
            if (item && itemSize > 0) {
                audioManager.writePcmChunk((const int16_t*)item, itemSize / sizeof(int16_t));
                vRingbufferReturnItem(g_pcmRingBuffer, item);
            } else if (waitingBytes == 0) {
                // Smooth recovery pause (re-arms clean buffer instead of machine-gun stuttering)
                isPrebuffered = false;
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }
    }
}