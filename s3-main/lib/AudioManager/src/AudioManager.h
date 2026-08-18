#pragma once

#include <Arduino.h>
#include <driver/i2s.h>
#include "audio_config.h"

namespace AudioState {
    enum { IDLE = 0, PLAYING = 1, ERROR = 2 };
}

class AudioManager {
public:
    AudioManager();

    bool begin();                       // Installs I2S TX + sets pins (BCLK/LRC/DIN)
    void setVolume(float v);            // 0.0f .. 1.0f (Q15 fixed-point accelerated)
    float getVolume() const;
    void stop();                        // Gracefully clears DMA buffers without hardware pop
    int  state() const;                 // AudioState

    // Called exclusively by TaskAudio on Core 1:
    bool playTone(uint16_t freqHz, uint16_t ms);                 // Smooth envelope beep (no click)
    bool playAlarm(const char* name);                            // Preset audio alarms
    bool playPcm(const int16_t* samples, size_t count);          // Plays TTS / raw PCM with Q15 gain

    // Streaming helper for direct chunks (e.g., Live WebSocket audio)
    size_t writePcmChunk(const int16_t* samples, size_t count);

private:
    void applyGain(int16_t* dst, const int16_t* src, size_t count);

    int        m_state;
    float      m_volume;
    int32_t    m_volQ15;                // Q15 fixed-point volume (0 .. 32767)
    i2s_port_t m_port;
    bool       m_initialized;
};