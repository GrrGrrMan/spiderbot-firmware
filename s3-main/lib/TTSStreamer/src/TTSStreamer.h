#pragma once

#include <Arduino.h>
#include "audio_config.h"

class TTSStreamer {
public:
    enum class FeedResult { OK, CHUNK_READY, FLOW_COMPLETE, ERROR, RESET };

    void begin();
    FeedResult feed(
        const String& flowId, uint16_t seq, uint16_t total, 
        const String& b64Payload, int16_t** outPcm, size_t* outSamples
    );
    void resetFlow();

private:
    static uint8_t b64Value(char c);
    static bool decodeBase64(const String& in, uint8_t* out, size_t maxOut, size_t& outLen);

    String   m_activeFlowId;
    uint16_t m_expectedTotal;
    bool     m_headerParsed;
    uint8_t  m_decodeBuffer[TTS_FRAME_MAX_B64];
    int16_t  m_pcmChunkBuffer[TTS_FRAME_MAX_B64 / 2];
};