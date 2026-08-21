#include "TTSStreamer.h"
#include "logger.h"

void TTSStreamer::begin() {
    m_activeFlowId = "";
    m_expectedTotal = 0;
    m_headerParsed = false;
}

uint8_t TTSStreamer::b64Value(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return 0xFF;
}

bool TTSStreamer::decodeBase64(const String& in, uint8_t* out, size_t maxOut, size_t& outLen) {
    outLen = 0;
    uint32_t acc = 0;
    uint8_t nBits = 0;
    for (size_t i = 0; i < in.length(); i++) {
        char c = in[i];
        if (c == '=') break;
        uint8_t v = b64Value(c);
        if (v == 0xFF) continue;
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

TTSStreamer::FeedResult TTSStreamer::feed(
    const String& flowId, uint16_t seq, uint16_t total, 
    const String& b64Payload, int16_t** outPcm, size_t* outSamples
) {
    if (b64Payload.length() == 0 || total == 0) return FeedResult::ERROR;

    // A new/different flow ID supersedes any previous assembly
    if (m_activeFlowId != flowId) {
        resetFlow();
        m_activeFlowId = flowId;
        m_expectedTotal = total;
    }

    size_t decodedBytes = 0;
    if (!decodeBase64(b64Payload, m_decodeBuffer, sizeof(m_decodeBuffer), decodedBytes)) {
        LOG_ERR("AUDIO TTS base64 decode failed (seq=%u)", seq);
        return FeedResult::ERROR;
    }

    size_t pcmOffset = 0;
    if (seq == 0) {
        // Strip 44-byte WAV header on the initial sequence slice
        if (decodedBytes > 44 && memcmp(m_decodeBuffer, "RIFF", 4) == 0) {
            pcmOffset = 44;
            m_headerParsed = true;
        }
    }

    size_t pcmBytes = (decodedBytes > pcmOffset) ? (decodedBytes - pcmOffset) : 0;
    *outSamples = pcmBytes / sizeof(int16_t);

    if (*outSamples > 0) {
        memcpy(m_pcmChunkBuffer, m_decodeBuffer + pcmOffset, pcmBytes);
        *outPcm = m_pcmChunkBuffer;
    } else {
        *outPcm = nullptr;
    }

    if (seq + 1 >= m_expectedTotal) {
        return FeedResult::FLOW_COMPLETE;
    }

    return (*outSamples > 0) ? FeedResult::CHUNK_READY : FeedResult::OK;
}

void TTSStreamer::resetFlow() {
    m_activeFlowId = "";
    m_expectedTotal = 0;
    m_headerParsed = false;
}