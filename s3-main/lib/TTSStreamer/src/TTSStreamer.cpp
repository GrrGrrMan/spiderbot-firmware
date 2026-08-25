#include "TTSStreamer.h"
#include "logger.h"

void TTSStreamer::begin() {
    m_activeFlowId = 0;
    m_expectedTotal = 0;
}

TTSStreamer::FeedResult TTSStreamer::feedBinary(
    uint32_t flowId, uint16_t seq, uint16_t total, 
    const uint8_t* pcmPayload, size_t pcmLength, 
    int16_t** outPcm, size_t* outSamples
) {
    if (pcmLength == 0 || total == 0) return FeedResult::ERROR;

    if (m_activeFlowId != flowId) {
        resetFlow();
        m_activeFlowId = flowId;
        m_expectedTotal = total;
    }

    *outSamples = pcmLength / sizeof(int16_t);

    if (*outSamples > 0) {
        // Direct memory copy (10x faster than Base64 decode)
        memcpy(m_pcmChunkBuffer, pcmPayload, pcmLength);
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
    m_activeFlowId = 0;
    m_expectedTotal = 0;
}