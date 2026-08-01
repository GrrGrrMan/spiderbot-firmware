#include "motor_v3.h"
#include "Motor/servos.h"
#include "Build/Log/logger.h"

#define V3_BUFFER_SIZE 64          
#define V3_PRE_ROLL_WATERMARK 3
#define V3_TICK_RATE_MS 20         

struct V3Keyframe {
    uint32_t seq;
    uint32_t duration_ms;
    int angles[CFG_SERVO_CHANNELS];
};

static V3Keyframe s_buffer[V3_BUFFER_SIZE];
static int s_head = 0;
static int s_tail = 0;
static int s_count = 0;

static float s_current_angles[CFG_SERVO_CHANNELS];
static float s_step_increments[CFG_SERVO_CHANNELS];

static uint32_t s_last_seq = 0;
static uint32_t s_last_packet_ms = 0;

enum V3State { STATE_STARVED, STATE_PREROLL, STATE_PLAYING };
static V3State s_state = STATE_STARVED;

// --- Lightweight JSON Parsers ---
static bool jsonGetInt(const String &payload, const char *key, long &out) {
    String token = "\"" + String(key) + "\"";
    int keyPos = payload.indexOf(token);
    if (keyPos == -1) return false;
    int colon = payload.indexOf(':', keyPos + token.length());
    if (colon == -1) return false;
    int start = colon + 1;
    while (start < (int)payload.length() && isspace(payload[start])) start++;
    int end = start;
    if (end < (int)payload.length() && payload[end] == '-') end++;
    while (end < (int)payload.length() && isdigit(payload[end])) end++;
    if (end <= start) return false;
    out = payload.substring(start, end).toInt();
    return true;
}

static int jsonParseAngles(const String &payload, int *angles, int maxAngles) {
    int keyPos = payload.indexOf("\"a\"");
    if (keyPos == -1) return 0;
    int start = payload.indexOf('[', keyPos + 3);
    int end = payload.indexOf(']', start + 1);
    if (start == -1 || end == -1) return -1;

    int count = 0;
    int index = start + 1;
    while (index < end && count < maxAngles) {
        while (index < end && (isspace(payload[index]) || payload[index] == ',')) index++;
        if (index >= end) break;
        int valueStart = index;
        if (payload[index] == '-') index++;
        if (index >= end || !isdigit(payload[index])) return -1;
        while (index < end && isdigit(payload[index])) index++;
        angles[count++] = payload.substring(valueStart, index).toInt();
    }
    return count;
}
// --------------------------------

void motor_v3_init() {
    servos_init();
    for (int i = 0; i < CFG_SERVO_CHANNELS; i++) {
        s_current_angles[i] = 90.0f;
        s_step_increments[i] = 0.0f;
    }
    LOG("[MOTOR V3] Unified Tick-Buffered Engine Initialized. Hardware Live.");
}

void motor_v3_handle_stream_json(const String &payload) {
    long seq = 0, dur = 0;
    if (!jsonGetInt(payload, "s", seq) || !jsonGetInt(payload, "d", dur)) return;

    if ((uint32_t)seq <= s_last_seq && s_last_seq != 0) return;
    s_last_seq = (uint32_t)seq;
    s_last_packet_ms = millis();

    // FIXED: Head-Drop Buffer Policy. Keep real-time responsiveness by ditching stale history!
    if (s_count >= V3_BUFFER_SIZE) {
        LOG("[MOTOR V3] Buffer Overflow! Dropping oldest stale frame to maintain real-time.");
        s_tail = (s_tail + 1) % V3_BUFFER_SIZE;
        s_count--; 
    }

    V3Keyframe frame;
    frame.seq = seq;
    frame.duration_ms = (uint32_t)dur;
    
    if (jsonParseAngles(payload, frame.angles, CFG_SERVO_CHANNELS) != CFG_SERVO_CHANNELS) return;

    s_buffer[s_head] = frame;
    s_head = (s_head + 1) % V3_BUFFER_SIZE;
    s_count++;

    if (s_state == STATE_STARVED) {
        s_state = STATE_PREROLL;
        LOG("[MOTOR V3] Stream detected. Buffering...");
    }
}

void motor_v3_handle() {
    uint32_t now = millis();
    static uint32_t s_last_tick = 0;
    static int s_ticks_remaining = 0; // Unifies hardware and frame playback

    // 1. Pre-Roll Logic
    if (s_state == STATE_PREROLL) {
        if (s_count >= V3_PRE_ROLL_WATERMARK || (s_count > 0 && now - s_last_packet_ms > 50)) {
            s_state = STATE_PLAYING;
            s_ticks_remaining = 0; // Force immediate pop
            LOG("[MOTOR V3] Pre-roll complete. Playback started.");
        }
    }

    // 2. Unified Hardware Execution & Frame Gate (Exactly 50Hz)
    // By nesting the frame logic INSIDE the 20ms tick gate, they can never drift out of phase.
    if (now - s_last_tick >= V3_TICK_RATE_MS) {
        s_last_tick = now;
        
        if (s_state == STATE_PLAYING) {
            
            // Pop new frame if the current one is finished rendering
            if (s_ticks_remaining <= 0) {
                if (s_count > 0) {
                    V3Keyframe target = s_buffer[s_tail];
                    s_tail = (s_tail + 1) % V3_BUFFER_SIZE;
                    s_count--;

                    // SAFEGUARD: Floor duration to prevent division by zero and micro-duration bursts
                    uint32_t safe_dur = target.duration_ms;
                    if (safe_dur < V3_TICK_RATE_MS) {
                        safe_dur = V3_TICK_RATE_MS; 
                    }
                    
                    s_ticks_remaining = safe_dur / V3_TICK_RATE_MS;
                    
                    for (int i = 0; i < CFG_SERVO_CHANNELS; i++) {
                        s_step_increments[i] = (target.angles[i] - s_current_angles[i]) / (float)s_ticks_remaining;
                    }
                } else {
                    s_state = STATE_STARVED;
                    LOG("[MOTOR V3] Buffer Starved. Holding position.");
                    for (int i = 0; i < CFG_SERVO_CHANNELS; i++) s_step_increments[i] = 0.0f;
                }
            }

            // Apply physical increment (Runs strictly once per 20ms tick)
            if (s_ticks_remaining > 0) {
                for (int i = 0; i < CFG_SERVO_CHANNELS; i++) {
                    s_current_angles[i] += s_step_increments[i];
                    
                    // PHASE 3 HARDWARE LIVE
                    servos_set_timed(i, (int)s_current_angles[i], 0); 
                }
                s_ticks_remaining--;
            }
        }
    }
}