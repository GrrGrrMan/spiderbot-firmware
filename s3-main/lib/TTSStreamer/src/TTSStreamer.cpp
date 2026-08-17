#include "TTSStreamer.h"
#include "logger.h"

void TTSStreamer::begin() {
    m_flow.wavBytes = nullptr;
    m_flow.wavSize = 0;
    m_flow.sampleRate = 0;
    m_flow.channels = 0;
    m_flow.bitsPerSample = 0;
    m_flow.dataOffset = 0;
    m_expectedTotal = 0;
    m_written = 0;
    m_capacity = 0;
    m_assembling = false;
    m_complete = false;
}

// --- base64 ----------------------------------------------------------------

uint8_t TTSStreamer::b64Value(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return 0xFF; // non-base64 / padding
}

bool TTSStreamer::decodeBase64(const String& in, uint8_t* out, size_t maxOut, size_t& outLen) {
    outLen = 0;
    uint32_t acc = 0;
    uint8_t nBits = 0;
    for (size_t i = 0; i < in.length(); i++) {
        char c = in[i];
        if (c == '=') break;             // padding
        uint8_t v = b64Value(c);
        if (v == 0xFF) return false;     // invalid char
        acc = (acc << 6) | v;
        nBits += 6;
        if (nBits >= 8) {
            nBits -= 8;
            if (outLen >= maxOut) return false;
            out[outLen++] = (uint8_t)((acc >> nBits) & 0xFF);
        }
    }
    return true;
}

// --- buffer allocation (PSRAM first, internal heap fallback) -------------

bool TTSStreamer::allocBuffer(size_t capacityBytes) {
    if (m_flow.wavBytes) free(m_flow.wavBytes);
    m_flow.wavBytes = nullptr;

    if (capacityBytes > TTS_MAX_FLOW_BYTES) return false;

    void* ptr = ps_malloc(capacityBytes);
    if (!ptr) ptr = malloc(capacityBytes);
    if (!ptr) return false;
    m_flow.wavBytes = (uint8_t*)ptr;
    m_flow.wavSize = 0;
    m_capacity = capacityBytes;
    m_written = 0;              // fresh buffer always starts at offset 0
    return true;
}
// --- WAV header parse -------------------------------------------------------

bool TTSStreamer::parseWavHeader() {
    const uint8_t* b = m_flow.wavBytes;
    if (m_flow.wavSize < 44) {
        LOG_ERR("AUDIO TTS WAV too small: %u bytes", (unsigned)m_flow.wavSize);
        return false;
    }
    if (memcmp(b, "RIFF", 4) != 0 || memcmp(b + 8, "WAVE", 4) != 0) {
        LOG_ERR("AUDIO TTS WAV bad RIFF/WAVE sig: '%.4s/%.4s' at 0/%u",
                (const char*)b, (const char*)(b + 8), (unsigned)m_flow.wavSize);
        return false;
    }

    size_t off = 12;
    bool haveFmt = false;
    uint32_t dataOffset = 0;
    uint16_t channelCount = 0, bitsPerSample = 0, audioFormat = 0;
    uint32_t sampleRate = 0;
    while (off + 8 <= m_flow.wavSize) {
        uint32_t chunkSize = (uint32_t)b[off + 4] | ((uint32_t)b[off + 5] << 8) |
                             ((uint32_t)b[off + 6] << 16) | ((uint32_t)b[off + 7] << 24);
        if (memcmp(b + off, "fmt ", 4) == 0 && !haveFmt) {
            audioFormat  = (uint16_t)(b[off + 8] | (b[off + 9] << 8));
            channelCount = (uint16_t)(b[off + 10] | (b[off + 11] << 8));
            sampleRate   = (uint32_t)b[off + 12] | ((uint32_t)b[off + 13] << 8) |
                           ((uint32_t)b[off + 14] << 16) | ((uint32_t)b[off + 15] << 24);
            bitsPerSample = (uint16_t)(b[off + 22] | (b[off + 23] << 8));
            haveFmt = true;
            if (audioFormat != 1) {
                LOG_ERR("AUDIO TTS WAV audioFormat=%u (want 1=PCM)", audioFormat);
                return false;
            }
        } else if (memcmp(b + off, "data", 4) == 0) {
            dataOffset = off + 8;
            break;
        }
        off += 8 + chunkSize + (chunkSize & 1);                 // pad to even
    }
    if (!haveFmt || dataOffset == 0 || bitsPerSample != 16) {
        LOG_ERR("AUDIO TTS WAV incomplete: haveFmt=%d dataOff=%u bits=%u",
                (int)haveFmt, (unsigned)dataOffset, bitsPerSample);
        return false;
    }
    if (sampleRate != AUDIO_SAMPLE_RATE || channelCount != 1) {
        LOG_ERR("AUDIO TTS WAV rejected: %luHz/%u ch (want %uHz/1)",
                (unsigned long)sampleRate, channelCount, (unsigned)AUDIO_SAMPLE_RATE);
        return false;
    }
    m_flow.channels      = channelCount;
    m_flow.sampleRate    = sampleRate;
    m_flow.bitsPerSample = bitsPerSample;
    m_flow.dataOffset    = dataOffset;
    return true;
}

// --- frame feeding -----------------------------------------------------------

TTSStreamer::FeedResult TTSStreamer::feed(const String& flowId, uint16_t seq, uint16_t total, const String& b64Payload) {
    if (b64Payload.length() == 0 || total == 0) return FeedResult::ERROR;

    // A new/different flow supersedes any previous assembly.
    if (!m_assembling || m_activeFlowId != flowId) {
        resetFlow();
        m_activeFlowId = flowId;
        m_expectedTotal = total;
        size_t capacity = ((size_t)total * b64Payload.length() * 3 / 4) + 64;
        if (!allocBuffer(capacity)) {
            LOG_ERR("AUDIO TTS PSRAM alloc failed for %u frames", total);
            return FeedResult::ERROR;
        }
        m_assembling = true;
    }

    if (seq >= m_expectedTotal) return FeedResult::ERROR;      // out of range

    size_t got = 0;
    if (!decodeBase64(b64Payload, m_flow.wavBytes + m_written, m_capacity - m_written, got)) {
        LOG_ERR("AUDIO TTS base64 decode failed (seq=%u)", seq);
        return FeedResult::ERROR;
    }
    m_written += got;
    m_flow.wavSize = m_written;

    if (seq + 1 == m_expectedTotal) {
        if (!parseWavHeader()) {
            LOG_ERR("AUDIO TTS WAV parse failed for flow '%s'", flowId.c_str());
            return FeedResult::ERROR;
        }
        m_complete = true;
        m_assembling = false;
        LOG_SYS("AUDIO TTS flow '%s' assembled (%u bytes, %uHz/%u ch/%u-bit)",
                flowId.c_str(), (unsigned)m_flow.wavSize, (unsigned)m_flow.sampleRate,
                m_flow.channels, m_flow.bitsPerSample);
        return FeedResult::FLOW_COMPLETE;
    }
    return FeedResult::OK;
}

void TTSStreamer::releaseFlow() {
    if (m_flow.wavBytes) free(m_flow.wavBytes);
    m_flow.wavBytes = nullptr;
    m_flow.wavSize = 0;
    m_flow.dataOffset = 0;
    m_capacity = 0;
    m_written = 0;              // CRITICAL: must reset so the next flow decodes
                                // from offset 0. Before this fix, the stale
                                // m_written from the previous flow made the next
                                // feed() decode into wavBytes+stale with a tiny
                                // remaining maxOut -> decodeBase64 overflow ->
                                // 'first audio works, second silent' bug.
    m_complete = false;
    m_assembling = false;
    m_activeFlowId = "";
}

void TTSStreamer::resetFlow() {
    releaseFlow();
}