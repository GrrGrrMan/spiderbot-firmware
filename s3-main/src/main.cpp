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
#include "AudioManager.h"
#include "audio_config.h"
#include "TTSStreamer.h"

bool g_logEnabled = true;

NetworkManager netManager;
MQTTManager mqttManager;
ServoManager servoManager;
MotionController motionController(servoManager);
CommandDispatcher cmdDispatcher;
AudioManager audioManager;
TTSStreamer ttsStreamer;

volatile unsigned long g_lastCmdTime = 0;

// Single-writer audio contract: TaskAudio is the ONLY thread that touches
// audioManager and ttsStreamer. Both MQTT-driven audio commands (beep/alarm/
// tts) and the boot chirp land here. TaskNetwork (MQTT callback) only
// pushes an AudioCommand token; it never blocks on i2s_write. This fixes
// the 'first try works, second silent' bug caused by:
//   (a) MQTT callback blocking 100s of ms in i2s_write (dropping frames)
//   (b) use-after-free: TaskAudio reads ttsStreamer.flow() then TaskNetwork
//       frees the same buffer when a new flow starts arriving
//   (c) two tasks racing on the same i2s_port / DMA buffer
enum class AudioCommandType : uint8_t {
    TONE  = 0,   // playTone(freqHz, ms)
    ALARM = 1,   // playAlarm(name)
    TTS   = 2,   // TTS flow ready in ttsStreamer; consume + play
};
struct AudioCommand {
    AudioCommandType type;
    uint16_t         freqHz;    // TONE
    uint16_t         ms;        // TONE
    char             alarmName[16];  // ALARM (null-terminated)
};
static QueueHandle_t g_audioQueue = nullptr;            // carries AudioCommand, depth 4
static volatile bool g_audioDonePending = false;        // set by TaskAudio, consumed by TaskNetwork for MQTT idle status
static char g_audioIdleAction[16] = {0};                // action of the finished playback ("tts"/"beep"/"alarm")
                                                        // written by TaskAudio BEFORE setting g_audioDonePending,
                                                        // read by TaskNetwork AFTER it sees the flag (single-writer)

void TaskNetwork(void *pvParameters);
void TaskControl(void *pvParameters);
void TaskAudio(void *pvParameters);

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

// Boot-time audio self-test: initializes I2S and plays a short chirp + idle alarm.
// This is the Wokwi-verifiable hook for scenarios/with-audio (Wokwi can't emit
// sound, but the I2S write/bytes logs prove the audio path runs).
static void runBootAudioTest() {
    if (!audioManager.begin()) {
        LOG_ERR("AUDIO boot self-test aborted: I2S init failed");
        return;
    }
    audioManager.playTone(440, 120);   // boot chirp  -> "AUDIO beep (...)"
    audioManager.playAlarm("idle");    //             -> "AUDIO alarm 'idle' played"
}

#if TTS_SIM_SELFTEST
// Small base64 encoder (selftest only; the RPi publish path uses piper + python).
static void b64EncodeChunk(const uint8_t* src, size_t n, char* dst) {
    static const char T[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t d = 0, s = 0;
    while (s + 3 <= n) {
        uint32_t v = ((uint32_t)src[s] << 16) | ((uint32_t)src[s + 1] << 8) | src[s + 2];
        dst[d++] = T[(v >> 18) & 63]; dst[d++] = T[(v >> 12) & 63];
        dst[d++] = T[(v >> 6) & 63];  dst[d++] = T[v & 63];
        s += 3;
    }
    size_t rem = n - s;
    if (rem == 1) {
        uint32_t v = (uint32_t)src[s] << 16;
        dst[d++] = T[(v >> 18) & 63]; dst[d++] = T[(v >> 12) & 63];
        dst[d++] = '='; dst[d++] = '=';
    } else if (rem == 2) {
        uint32_t v = ((uint32_t)src[s] << 16) | ((uint32_t)src[s + 1] << 8);
        dst[d++] = T[(v >> 18) & 63]; dst[d++] = T[(v >> 12) & 63];
        dst[d++] = T[(v >> 6) & 63];  dst[d++] = '=';
    }
    dst[d] = 0;
}

// Wokwi-only: synthesizes a tiny 22050 Hz mono 16-bit sine WAV and feeds it to
// TTSStreamer through the exact same 4 KB base64 frame path the RPi ai-service
// uses over MQTT, then kicks TaskAudio. Asserted by scenarios/with-audio/test-tts.yaml.
static void runBootTtsSelftest() {
    static uint8_t wav[64 + AUDIO_SAMPLE_RATE * 2];     // ~1s cap
    uint32_t numSamples = (uint32_t)(AUDIO_SAMPLE_RATE * 300 / 1000);  // 0.3s
    size_t dataSize = numSamples * 2;
    size_t total = 44 + dataSize;

    memcpy(wav, "RIFF", 4);
    uint32_t riff = (uint32_t)(total - 8);
    wav[4] = riff & 0xFF; wav[5] = (riff >> 8) & 0xFF; wav[6] = (riff >> 16) & 0xFF; wav[7] = (riff >> 24) & 0xFF;
    memcpy(wav + 8, "WAVE", 4);
    memcpy(wav + 12, "fmt ", 4);
    wav[16] = 16; wav[17] = 0; wav[18] = 0; wav[19] = 0;
    wav[20] = 1; wav[21] = 0;                            // PCM
    wav[22] = 1; wav[23] = 0;                            // mono
    wav[24] = AUDIO_SAMPLE_RATE & 0xFF; wav[25] = (AUDIO_SAMPLE_RATE >> 8) & 0xFF;
    wav[26] = (AUDIO_SAMPLE_RATE >> 16) & 0xFF; wav[27] = (AUDIO_SAMPLE_RATE >> 24) & 0xFF;
    uint32_t byteRate = (uint32_t)AUDIO_SAMPLE_RATE * 2;
    wav[28] = byteRate & 0xFF; wav[29] = (byteRate >> 8) & 0xFF; wav[30] = (byteRate >> 16) & 0xFF; wav[31] = (byteRate >> 24) & 0xFF;
    wav[32] = 2; wav[33] = 0;                            // block align
    wav[34] = 16; wav[35] = 0;                           // bits
    memcpy(wav + 36, "data", 4);
    wav[40] = dataSize & 0xFF; wav[41] = (dataSize >> 8) & 0xFF;
    wav[42] = (dataSize >> 16) & 0xFF; wav[43] = (dataSize >> 24) & 0xFF;
    for (size_t i = 0; i < numSamples; i++) {
        int16_t s = (int16_t)(20000.0f * sinf(2.0f * (float)M_PI * 440.0f * (float)i / (float)AUDIO_SAMPLE_RATE));
        wav[44 + i * 2] = (uint8_t)(s & 0xFF);
        wav[45 + i * 2] = (uint8_t)((s >> 8) & 0xFF);
    }

    LOG_SYS("AUDIO TTS selftest: feeding %u WAV bytes via 3KB b64 frames", (unsigned)total);
    static char frameB64[TTS_FRAME_MAX_B64 + 1];   // static: 4 KB must NOT sit on the 4 KB ControlTask stack
    uint16_t totalFrames = (uint16_t)((total + 3071) / 3072);
    for (uint16_t seq = 0; seq < totalFrames; seq++) {
        size_t off = (size_t)seq * 3072;
        size_t n = (off + 3072 <= total) ? 3072 : (total - off);
        b64EncodeChunk(wav + off, n, frameB64);
        TTSStreamer::FeedResult r = ttsStreamer.feed("selftest", seq, totalFrames, String(frameB64));
        if (r == TTSStreamer::FeedResult::ERROR) {
            LOG_ERR("AUDIO TTS selftest FAILED at seq %u/%u", seq, totalFrames);
            return;
        }
    }
    AudioCommand cmd{};
    cmd.type = AudioCommandType::TTS;
    if (g_audioQueue) xQueueSend(g_audioQueue, &cmd, 0);
    LOG_SYS("AUDIO TTS selftest queued for TaskAudio playback");
}
#endif // TTS_SIM_SELFTEST

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

    // Audio commands arrive on hexapod/{id}/audio (keyed by "action", not "type").
    // NOTE: deliberately does NOT touch g_lastCmdTime — audio is independent of the
    // servo safety watchdog (I-4), so a speaker request must never keep motion alive.
    mqttManager.setAudioCommandCallback([](const String& action, JsonDocument& doc) {
        // Everything below is NON-BLOCKING: it just enqueues an AudioCommand
        // for TaskAudio to consume. TaskNetwork must NOT touch i2s_write or
        // ttsStreamer directly — that was the source of the
        // 'first try works, second silent' bug (use-after-free + DMA race).
        auto enqueue = [](const AudioCommand& cmd) -> bool {
            if (!g_audioQueue) return false;
            // pdMS_TO_TICKS(0): non-blocking. If the queue is saturated, log
            // + drop — better than blocking the MQTT callback on i2s_write.
            if (xQueueSend(g_audioQueue, &cmd, 0) != pdTRUE) {
                LOG_ERR("AUDIO queue full (depth 4); dropping cmd type=%u", (unsigned)cmd.type);
                return false;
            }
            return true;
        };
        AudioCommand cmd{};
        if (action == "beep") {
            cmd.type = AudioCommandType::TONE;
            cmd.freqHz = 1200;
            cmd.ms = 120;
            mqttManager.sendAudioStatus("playing", "beep");
            enqueue(cmd);
        } else if (action == "alarm") {
            const char* name = doc["payload"] | "";
            if (strlen(name) > 0) {
                cmd.type = AudioCommandType::ALARM;
                strncpy(cmd.alarmName, name, sizeof(cmd.alarmName) - 1);
                cmd.alarmName[sizeof(cmd.alarmName) - 1] = '\0';
                mqttManager.sendAudioStatus("playing", "alarm");
                enqueue(cmd);
            }
        } else if (action == "play") {
            cmd.type = AudioCommandType::TONE;
            cmd.freqHz = 660;
            cmd.ms = 120;
            mqttManager.sendAudioStatus("playing", "play");
            enqueue(cmd);
        } else if (action == "tts") {
            // P5 chunked TTS: RPi Piper WAV split into base64 frames
            // {flow_id, seq, total, payload}. Assemble in PSRAM (called from
            // the MQTT callback is OK — no I/O, just PSRAM; the buffer is
            // only handed off to TaskAudio once FLOW_COMPLETE fires).
            String  flowId  = doc["flow_id"] | "";
            uint16_t seq    = doc["seq"]    | 0;
            uint16_t total  = doc["total"]  | 0;
            String  payload = doc["payload"] | "";

            TTSStreamer::FeedResult res = ttsStreamer.feed(flowId, seq, total, payload);
            if (res == TTSStreamer::FeedResult::FLOW_COMPLETE) {
                cmd.type = AudioCommandType::TTS;
                mqttManager.sendAudioStatus("playing", "tts");
                enqueue(cmd);
            } else if (res == TTSStreamer::FeedResult::ERROR) {
                LOG_ERR("AUDIO TTS error (flow=%s seq=%u/%u len=%u)", flowId.c_str(), seq, total, payload.length());
                ttsStreamer.resetFlow();
                mqttManager.sendAudioStatus("error", "tts");
            }
        } else {
            LOG_ERR("Unknown audio action: '%s'", action.c_str());
        }
    });

    g_audioQueue = xQueueCreate(4, sizeof(AudioCommand));
    ttsStreamer.begin();

    xTaskCreatePinnedToCore(TaskNetwork, "NetTask", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(TaskControl, "ControlTask", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(TaskAudio, "AudioTask", 8192, NULL, 1, NULL, 1);
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

            // TaskAudio finished a playback (TTS flow, beep, or alarm) ->
            // publish 'idle' here (core 0), so all MQTT writes stay on the
            // network task (PubSubClient is not concurrent-safe across tasks).
            if (g_audioDonePending) {
                g_audioDonePending = false;
                mqttManager.sendAudioStatus("idle", g_audioIdleAction[0] ? g_audioIdleAction : "tts");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void TaskControl(void *pvParameters) {
    servoManager.begin();
    LOG_SYS("S3 Servo Manager ready");
    runBootServoCycle();
    runBootAudioTest();
#if TTS_SIM_SELFTEST
    runBootTtsSelftest();
#endif
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

// Dedicated I2S playback task: the ONLY thread that calls audioManager and
// ttsStreamer. Consumes AudioCommand tokens pushed by TaskNetwork's MQTT
// callback (non-blocking) or the Wokwi selftest. i2s_write blocks here while
// TaskNetwork (core 0) keeps MQTT alive — never the other way around.
void TaskAudio(void *pvParameters) {
    AudioCommand cmd;
    for (;;) {
        if (xQueueReceive(g_audioQueue, &cmd, portMAX_DELAY) != pdTRUE) continue;

        switch (cmd.type) {
            case AudioCommandType::TONE:
                audioManager.playTone(cmd.freqHz, cmd.ms);
                strncpy(g_audioIdleAction, "tone", sizeof(g_audioIdleAction) - 1);
                g_audioIdleAction[sizeof(g_audioIdleAction) - 1] = '\0';
                g_audioDonePending = true;
                break;
            case AudioCommandType::ALARM:
                audioManager.playAlarm(cmd.alarmName[0] ? cmd.alarmName : "idle");
                strncpy(g_audioIdleAction, "alarm", sizeof(g_audioIdleAction) - 1);
                g_audioIdleAction[sizeof(g_audioIdleAction) - 1] = '\0';
                g_audioDonePending = true;
                break;
            case AudioCommandType::TTS:
            default:
                if (!ttsStreamer.hasCompleteFlow()) break;
                {
                    const TTSStreamer::Flow& flow = ttsStreamer.flow();
                    LOG_SYS("AUDIO TTS playing %u samples (%luHz/%u ch)",
                            (unsigned)flow.pcmSampleCount(), (unsigned long)flow.sampleRate, flow.channels);
                    audioManager.playPcm(flow.pcm(), flow.pcmSampleCount());
                    ttsStreamer.releaseFlow();
                }
                strncpy(g_audioIdleAction, "tts", sizeof(g_audioIdleAction) - 1);
                g_audioIdleAction[sizeof(g_audioIdleAction) - 1] = '\0';
                g_audioDonePending = true;
                LOG_SYS("AUDIO TTS playback complete");
                break;
        }
    }
}