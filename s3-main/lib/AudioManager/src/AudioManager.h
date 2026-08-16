#pragma once

#include <Arduino.h>
#include <driver/i2s.h>
#include "audio_config.h"

// Audio playback states (mirrored onto hexapod/{id}/audio/status).
namespace AudioState {
    enum { IDLE = 0, PLAYING = 1, ERROR = 2 };
}

// Drives the MAX98357 I2S amplifier from the ESP32-S3.
// Uses the legacy IDF i2s driver (deterministic pin mapping on Arduino core 2.0.x).
// Volume is a SOFTWARE gain applied to samples before i2s_write —
// the MAX98357 has no hardware volume control.
class AudioManager {
public:
    AudioManager();

    bool begin();                       // install I2S TX + set pins (BCLK/LRC/DIN)
    void setVolume(float v);            // 0.0 .. 1.0
    float getVolume() const;
    void stop();
    int  state() const;                 // AudioState

    bool playTone(uint16_t freqHz, uint16_t ms);                 // sine beep
    bool playAlarm(const char* name);                            // "startle" | "curious" | "idle"
    bool playPcm(const int16_t* samples, size_t count);          // raw PCM w/ gain (TTS path)

private:
    size_t writePcm(const int16_t* samples, size_t count);       // applies gain, i2s_write

    int      m_state;
    float    m_volume;
    i2s_port_t m_port;
    bool     m_initialized;
};