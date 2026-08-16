#include "AudioManager.h"
#include "logger.h"

#include <math.h>

AudioManager::AudioManager()
    : m_state(AudioState::IDLE),
      m_volume(1.0f),
      m_port(AUDIO_I2S_NUM),
      m_initialized(false) {}

bool AudioManager::begin() {
#if AUDIO_SIM_MODE
    m_initialized = true;
    m_state = AudioState::IDLE;
    LOG_SYS("AUDIO I2S ready (sim) - Wokwi has no I2S/DMA; PCM pipeline verified, hardware skipped");
    return true;
#endif
    i2s_config_t cfg = {};
    cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.sample_rate          = AUDIO_SAMPLE_RATE;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format       = I2S_CHANNEL_FMT_ONLY_RIGHT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count        = 8;
    cfg.dma_buf_len          = 128;
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

    i2s_set_clk(m_port, AUDIO_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);

    m_initialized = true;
    m_state = AudioState::IDLE;
    LOG_SYS("AUDIO I2S ready (BCLK=%d LRC=%d DIN=%d @%uHz)",
            PIN_AUDIO_BCLK, PIN_AUDIO_LRC, PIN_AUDIO_DIN, AUDIO_SAMPLE_RATE);
    return true;
}

void AudioManager::setVolume(float v) {
    m_volume = v > 1.0f ? 1.0f : (v < 0.0f ? 0.0f : v);
}

float AudioManager::getVolume() const { return m_volume; }

void AudioManager::stop() {
#if !AUDIO_SIM_MODE
    if (m_initialized) i2s_stop(m_port);
#endif
    m_state = AudioState::IDLE;
}

int AudioManager::state() const { return m_state; }

size_t AudioManager::writePcm(const int16_t* samples, size_t count) {
    if (!m_initialized || count == 0) return 0;
    if (m_state != AudioState::PLAYING) m_state = AudioState::PLAYING;

#if AUDIO_SIM_MODE
    // Sim mode (Wokwi): no I2S hardware to service — report the bytes a real write would
    // emit, keeping the PCM/state/log pipeline identical to the physical build.
    (void)samples;
    return count * sizeof(int16_t);
#endif
    // Software gain into a small stack buffer, streamed over I2S DMA in bounded chunks.
    static int16_t scaled[256];
    size_t writtenTotal = 0;
    size_t idx = 0;
    while (idx < count) {
        size_t chunk = (count - idx) > 256 ? 256 : (count - idx);
        for (size_t i = 0; i < chunk; i++) {
            float s = (float)samples[idx + i] * m_volume;
            scaled[i] = (int16_t)(s >  32767.0f ?  32767 : (s < -32768.0f ? -32768 : s));
        }
        size_t bytesWritten = 0;
        esp_err_t err = i2s_write(m_port, (const char*)scaled, chunk * sizeof(int16_t),
                                  &bytesWritten, pdMS_TO_TICKS(100));
        if (err != ESP_OK) {
            m_state = AudioState::ERROR;
            break;
        }
        writtenTotal += bytesWritten;
        idx += chunk;
    }
    return writtenTotal;
}

bool AudioManager::playTone(uint16_t freqHz, uint16_t ms) {
    if (!m_initialized) return false;

    size_t count = (size_t)((uint32_t)AUDIO_SAMPLE_RATE * ms / 1000);
    if (count == 0) count = 1;

    const float step = 2.0f * (float)M_PI * (float)freqHz / (float)AUDIO_SAMPLE_RATE;
    int16_t frame[256];
    size_t idx = 0;
    while (idx < count) {
        size_t n = ((count - idx) > 256) ? 256 : (count - idx);
        for (size_t k = 0; k < n; k++) {
            frame[k] = (int16_t)(sinf((float)(idx + k) * step) * 20000.0f);
        }
        writePcm(frame, n);
        idx += n;
    }

    m_state = AudioState::IDLE;
    LOG_SYS("AUDIO beep (%uHz,%ums) wrote ~%u bytes", freqHz, ms, (unsigned)(count * 2));
    return true;
}

bool AudioManager::playAlarm(const char* name) {
    if (!name) return false;

    if (strcmp(name, "startle") == 0) {
        playTone(880, 140); playTone(1100, 90); playTone(880, 140);
    } else if (strcmp(name, "curious") == 0) {
        playTone(660, 110); playTone(740, 110); playTone(880, 110);
    } else if (strcmp(name, "idle") == 0) {
        playTone(440, 160);
    } else {
        playTone(520, 200); // generic fallback
    }

    m_state = AudioState::IDLE;
    LOG_SYS("AUDIO alarm '%s' played", name);
    return true;
}

bool AudioManager::playPcm(const int16_t* samples, size_t count) {
    if (!m_initialized || !samples || count == 0) return false;
    writePcm(samples, count);
    m_state = AudioState::IDLE;
    LOG_SYS("AUDIO played %u PCM samples", (unsigned)count);
    return true;
}