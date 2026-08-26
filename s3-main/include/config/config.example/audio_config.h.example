#pragma once

// MAX98357 I2S amplifier pins (PLAN.md section 4 hardware pin map).
#define PIN_AUDIO_BCLK 40
#define PIN_AUDIO_LRC  39
#define PIN_AUDIO_DIN  38

// I2S stream format (matches the RPi Piper TTS spec: 16-bit 22050 Hz mono).
#define AUDIO_SAMPLE_RATE  22050
#define AUDIO_BITS         16
#define AUDIO_CHANNELS     1
#define AUDIO_I2S_NUM      I2S_NUM_0

// ── Audio Power & Volume Tuning ─────────────────────────────────────────────
#define AUDIO_DEFAULT_VOLUME   0.35f   // 35% baseline: keeps MAX98357 current draw low (< 300mA)
#define AUDIO_TONE_AMPLITUDE   6000.0f // Reduced from 20000.0f (eliminates startup audio current inrush)

#ifndef AUDIO_SIM_MODE
#define AUDIO_SIM_MODE 0
#endif

#ifndef TTS_FRAME_MAX_B64
#define TTS_FRAME_MAX_B64 4096
#endif

#ifndef TTS_MAX_FLOW_BYTES
#define TTS_MAX_FLOW_BYTES (2 * 1024 * 1024)
#endif

#ifndef TTS_SIM_SELFTEST
#define TTS_SIM_SELFTEST 0
#endif