#include "AudioManager.h"
#include "logger.h"
#include <math.h>
#include <string.h>

#define DMA_CHUNK_SAMPLES 256

AudioManager::AudioManager()
    : m_state(AudioState::IDLE),
      m_volume(AUDIO_DEFAULT_VOLUME),
      m_volQ15((int32_t)(AUDIO_DEFAULT_VOLUME * 32767.0f)),
      m_port(AUDIO_I2S_NUM),
      m_initialized(false) {}

bool AudioManager::begin() {
#if AUDIO_SIM_MODE
    m_initialized = true;
    m_state = AudioState::IDLE;
    LOG_SYS("AUDIO I2S ready (sim) - Wokwi mode");
    return true;
#endif

    i2s_config_t cfg = {};
    cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.sample_rate          = AUDIO_SAMPLE_RATE;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count        = 8;
    cfg.dma_buf_len          = 512;
    cfg.use_apll             = true;
    cfg.tx_desc_auto_clear   = true;

    esp_err_t err = i2s_driver_install(m_port, &cfg, 0, NULL);
    if (err != ESP_OK) {
        LOG_ERR("AUDIO i2s_driver_install failed (%d)", (int)err);
        m_state = AudioState::ERROR;
        return false;
    }

    i2s_pin_config_t pins = {
        .bck_io_num   = PIN_AUDIO_BCLK,
        .ws_io_num    = PIN_AUDIO_LRC,
        .data_out_num = PIN_AUDIO_DIN,
        .data_in_num  = I2S_PIN_NO_CHANGE,
    };
    err = i2s_set_pin(m_port, &pins);
    if (err != ESP_OK) {
        LOG_ERR("AUDIO i2s_set_pin failed (%d)", (int)err);
        m_state = AudioState::ERROR;
        return false;
    }

    i2s_set_clk(m_port, AUDIO_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);

    m_initialized = true;
    m_state = AudioState::IDLE;
    setVolume(AUDIO_DEFAULT_VOLUME);

    LOG_SYS("AUDIO I2S ready (Vol: %.0f%%, BCLK=%d LRC=%d DIN=%d @%uHz)",
            m_volume * 100.0f, PIN_AUDIO_BCLK, PIN_AUDIO_LRC, PIN_AUDIO_DIN, AUDIO_SAMPLE_RATE);
    return true;
}

void AudioManager::setVolume(float v) {
    m_volume = v > 1.0f ? 1.0f : (v < 0.0f ? 0.0f : v);
    m_volQ15 = (int32_t)(m_volume * 32767.0f);
}

float AudioManager::getVolume() const { return m_volume; }

int AudioManager::state() const { return m_state; }

void AudioManager::stop() {
    m_state = AudioState::IDLE;
}

void AudioManager::applyGain(int16_t* dst, const int16_t* src, size_t count) {
    if (m_volQ15 >= 32767) {
        memcpy(dst, src, count * sizeof(int16_t));
        return;
    }
    if (m_volQ15 <= 0) {
        memset(dst, 0, count * sizeof(int16_t));
        return;
    }
    for (size_t i = 0; i < count; i++) {
        dst[i] = (int16_t)(((int32_t)src[i] * m_volQ15) >> 15);
    }
}

size_t AudioManager::writePcmChunk(const int16_t* samples, size_t count) {
    if (!m_initialized || !samples || count == 0) return 0;
    m_state = AudioState::PLAYING;

#if AUDIO_SIM_MODE
    return count * sizeof(int16_t);
#endif

    int16_t stereoBuffer[DMA_CHUNK_SAMPLES * 2];
    size_t writtenTotal = 0;
    size_t idx = 0;

    while (idx < count) {
        size_t chunk = (count - idx) > DMA_CHUNK_SAMPLES ? DMA_CHUNK_SAMPLES : (count - idx);

        for (size_t i = 0; i < chunk; i++) {
            int16_t sample = samples[idx + i];
            if (m_volQ15 < 32767) {
                sample = (int16_t)(((int32_t)sample * m_volQ15) >> 15);
            }
            stereoBuffer[i * 2]     = sample;
            stereoBuffer[i * 2 + 1] = sample;
        }

        size_t bytesWritten = 0;
        esp_err_t err = i2s_write(
            m_port,
            stereoBuffer,
            chunk * 2 * sizeof(int16_t),
            &bytesWritten,
            pdMS_TO_TICKS(100)
        );

        if (err != ESP_OK) {
            m_state = AudioState::ERROR;
            break;
        }
        writtenTotal += (bytesWritten / 2);
        idx += chunk;
    }
    return writtenTotal;
}

bool AudioManager::playPcm(const int16_t* samples, size_t count) {
    if (!m_initialized || !samples || count == 0) return false;
    writePcmChunk(samples, count);
    m_state = AudioState::IDLE;
    return true;
}

bool AudioManager::playTone(uint16_t freqHz, uint16_t ms) {
    if (!m_initialized) return false;

    size_t totalSamples = (size_t)((uint32_t)AUDIO_SAMPLE_RATE * ms / 1000);
    if (totalSamples == 0) return true;

    size_t rampSamples = (AUDIO_SAMPLE_RATE * 5) / 1000;
    if (rampSamples > totalSamples / 2) rampSamples = totalSamples / 2;

    const float step = 2.0f * (float)M_PI * (float)freqHz / (float)AUDIO_SAMPLE_RATE;
    int16_t frame[DMA_CHUNK_SAMPLES];

    size_t generated = 0;
    while (generated < totalSamples) {
        size_t n = (totalSamples - generated) > DMA_CHUNK_SAMPLES ? DMA_CHUNK_SAMPLES : (totalSamples - generated);

        for (size_t k = 0; k < n; k++) {
            size_t currentSample = generated + k;
            float sample = sinf((float)currentSample * step) * AUDIO_TONE_AMPLITUDE; // Safe lower amplitude

            if (currentSample < rampSamples) {
                sample *= ((float)currentSample / (float)rampSamples);
            } else if (currentSample > (totalSamples - rampSamples)) {
                sample *= ((float)(totalSamples - currentSample) / (float)rampSamples);
            }
            frame[k] = (int16_t)sample;
        }

        writePcmChunk(frame, n);
        generated += n;

        taskYIELD();
    }

    m_state = AudioState::IDLE;
    return true;
}

bool AudioManager::playAlarm(const char* name) {
    if (!name) return false;

    if (strcmp(name, "startle") == 0) {
        playTone(880, 100); playTone(1100, 70); playTone(880, 100);
    } else if (strcmp(name, "curious") == 0) {
        playTone(660, 80); playTone(740, 80); playTone(880, 80);
    } else if (strcmp(name, "idle") == 0) {
        playTone(440, 100);
    } else {
        playTone(520, 120);
    }

    m_state = AudioState::IDLE;
    return true;
}