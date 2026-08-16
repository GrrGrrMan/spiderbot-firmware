#pragma once

// MAX98357 I2S amplifier pins (PLAN.md section 4 hardware pin map).
// Free on the S3 (SDA/SCL=41/42, OE=13 are taken by the PCA9685 bus).
#define PIN_AUDIO_BCLK 40
#define PIN_AUDIO_LRC  39
#define PIN_AUDIO_DIN  38

// I2S stream format (matches the RPi Piper TTS spec: 16-bit 22050 Hz mono).
#define AUDIO_SAMPLE_RATE  22050
#define AUDIO_BITS         16
#define AUDIO_CHANNELS     1
#define AUDIO_I2S_NUM      I2S_NUM_0

// Wokwi does NOT emulate the ESP32 I2S/DMA peripheral (github.com/wokwi/wokwi-features#213),
// so i2s_write never completes there -> the core blocks with interrupts masked -> IWDT panic.
// AUDIO_SIM_MODE=1 (Wokwi): verify the PCM/action/status pipeline, skip the I2S hardware.
// AUDIO_SIM_MODE=0: drive the real MAX98357 over I2S. Flip to 0 before physical deploy.
// (Guard lets the Wokwi scenario build pass -DAUDIO_SIM_MODE=1 without editing this file.)
#ifndef AUDIO_SIM_MODE
#define AUDIO_SIM_MODE 0
#endif

// P5 chunked TTS transfer (RPi ai-service -> S3).
// Piper WAV (22050 Hz mono 16-bit) is split into base64 frames <= 4 KB each
// because PubSubClient's buffer is a uint16_t-sized byte array (cap ~64 KB)
// and we keep the RX buffer at 8 KB so the network task never stalls.
#ifndef TTS_FRAME_MAX_B64
#define TTS_FRAME_MAX_B64 4096
#endif
#ifndef TTS_MAX_FLOW_BYTES
#define TTS_MAX_FLOW_BYTES (2 * 1024 * 1024)   // ~45 s of 22050 Hz mono speech
#endif

// Wokwi-only: run a synthetic chunked TTS flow through the full
// MQTT-frame + assembler + TaskAudio pipeline at boot (no broker needed).
// Enabled in the scenario build via -DTTS_SIM_SELFTEST=1.
#ifndef TTS_SIM_SELFTEST
#define TTS_SIM_SELFTEST 0
#endif