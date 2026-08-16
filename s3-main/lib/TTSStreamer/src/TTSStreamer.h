#pragma once

#include <Arduino.h>
#include "audio_config.h"

// P5 chunked TTS assembly (hexapod/{id}/audio, action:"tts").
// RPi ai-service (Piper) segments a 22050 Hz mono 16-bit WAV into base64
// frames and publishes:
//   { "action":"tts", "flow_id":"<id>", "seq":<0..total-1>, "total":<n>,
//     "payload":"<base64 slice <= 4KB>" }
// This class decodes each slice into a PSRAM buffer, and when seq==total-1
// parses the WAV header so the caller can stream PCM to the I2S amp.
// All RAM is PSRAM (ESP32-S3); playback itself is handled by TaskAudio,
// never inside the MQTT callback (PubSubClient buffer + I2S must not block
// the network task).
class TTSStreamer {
public:
    enum class FeedResult { OK, FLOW_COMPLETE, ERROR, RESET };

    struct Flow {
        uint8_t* wavBytes;     // complete WAV (header + PCM) in PSRAM
        size_t   wavSize;      // total WAV bytes
        uint32_t sampleRate;
        uint16_t channels;
        uint16_t bitsPerSample;

        const int16_t* pcm() const {
            // PCM section offset is remembered as wavBytes + m_dataOffset
            return (const int16_t*)(wavBytes + dataOffset);
        }
        size_t pcmSampleCount() const {
            return (wavSize - dataOffset) / (bitsPerSample / 8);
        }
        size_t dataOffset;
    };

    void begin();

    // Feed one frame. On FLOW_COMPLETE a playable Flow has been assembled.
    FeedResult feed(const String& flowId, uint16_t seq, uint16_t total, const String& b64Payload);

    bool hasCompleteFlow() const { return m_complete; }
    const Flow& flow() const { return m_flow; }
    bool takeFlow() { m_complete = false; return true; } // caller has a ref to Flow
    void releaseFlow();   // free the assembled buffer
    void resetFlow();     // abort the current (incomplete) assembly

private:
    static uint8_t b64Value(char c);
    static bool decodeBase64(const String& in, uint8_t* out, size_t maxOut, size_t& outLen);
    bool parseWavHeader();
    bool allocBuffer(size_t capacityBytes);

    Flow m_flow;
    String m_activeFlowId;
    uint16_t m_expectedTotal;
    size_t m_written;         // bytes written into m_flow.wavBytes so far
    size_t m_capacity;        // allocated size of m_flow.wavBytes
    bool m_assembling;
    bool m_complete;
};