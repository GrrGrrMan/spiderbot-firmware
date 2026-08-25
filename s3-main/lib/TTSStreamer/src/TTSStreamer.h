#pragma once

#include <Arduino.h>
#include "audio_config.h"

class TTSStreamer {
public:
    enum class FeedResult { OK, CHUNK_READY, FLOW_COMPLETE, ERROR, RESET };

    void begin();
    FeedResult feedBinary(
        uint32_t flowId, uint16_t seq, uint16_t total, 
        const uint8_t* pcmPayload, size_t pcmLength, 
        int16_t** outPcm, size_t* outSamples
    );
    void resetFlow();
    bool isActive() const { return m_activeFlowId != 0; }

private:
    uint32_t m_activeFlowId;
    uint16_t m_expectedTotal;
    int16_t  m_pcmChunkBuffer[TTS_FRAME_MAX_B64 / 2]; // Fits 4096 bytes natively aligned
};