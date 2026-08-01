#ifdef FEATURE_RGB

#include "led_pca.h"
#include "Build/config/target_config.h"
#include "Build/Log/logger.h"
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
// led_pca.cpp — Multi-RGB-LED controller via PCA9685.
//
// Pattern animation is driven by led_pca_handle(), called from led_handle()
// every loop() iteration.  No FreeRTOS tasks are created — patterns are
// cooperative state machines that fire only when their deadline passes.
//
// This eliminates:
//   • 2 × 2048-byte FreeRTOS task stacks
//   • 2 context switches per blink cycle
//   • The s_args race (single static PatternArgs overwritten on rapid restarts)
//   • Blocking led_pca_fade() (now non-blocking, ticked in handle)
//   • 15 I²C writes per chase step (reduced to 6 with dirty prev/next logic)
//
// Tradeoff: pattern timing is now subject to loop() jitter.  At the MQTT +
// OTA load levels in this firmware, jitter is <5 ms — imperceptible for LEDs.
// ─────────────────────────────────────────────────────────────────────────────

extern Adafruit_PWMServoDriver pwm; // defined in Motor/servos.cpp

// ── Helpers ───────────────────────────────────────────────────────────────
static uint16_t toPwm(uint8_t duty)
{
    uint16_t val = (uint16_t)duty * 16;
    return CFG_RGB_COMMON_ANODE ? (4095 - val) : val;
}

static void setChannel(uint8_t ch, uint8_t duty)
{
    pwm.setPWM(ch, 0, toPwm(duty));
}

static void setLed(uint8_t id, uint8_t r, uint8_t g, uint8_t b)
{
    if (id >= CFG_RGB_LED_COUNT)
        return;
    uint8_t base = CFG_RGB_START_CHANNEL + id * 3;
    setChannel(base, r);
    setChannel(base + 1, g);
    setChannel(base + 2, b);
}

// ── Retained colour state (needed for fade from-current) ─────────────────
static uint8_t s_r[CFG_RGB_LED_COUNT] = {};
static uint8_t s_g[CFG_RGB_LED_COUNT] = {};
static uint8_t s_b[CFG_RGB_LED_COUNT] = {};

// ── One-shot fade state ───────────────────────────────────────────────────
// Activated by led_pca_fade().  Ticked in led_pca_handle().
// Calling led_pca_pattern_* while a fade is active is safe — pattern_stop()
// clears both.  Running both simultaneously on the same LED is undefined;
// stop the pattern first.
struct FadeState
{
    bool active; // zero-initialised below → false at startup
    uint8_t id;
    uint8_t fromR, fromG, fromB;
    uint8_t toR, toG, toB;
    uint32_t startMs, durationMs;
};
static FadeState s_fade = {};

// ── Pattern state machine ─────────────────────────────────────────────────
enum class PatternType : uint8_t
{
    NONE,
    BLINK,
    PULSE,
    CHASE
};

struct PatternState
{
    PatternType type; // zero-initialised below → NONE (==0)
    uint8_t id;
    uint8_t r, g, b;
    uint32_t ms;       // on_ms (blink) | fade_ms (pulse) | gap_ms (chase)
    uint32_t ms2;      // off_ms (blink only)
    uint32_t nextTick; // millis() deadline for next blink/chase step
    bool phase;        // blink: false=LED on, true=LED off
    uint8_t chaseIdx;
    uint32_t fadeStart;
    bool fadingUp; // true = ramping toward colour, false = back to 0
    uint8_t lastR[CFG_RGB_LED_COUNT];
    uint8_t lastG[CFG_RGB_LED_COUNT];
    uint8_t lastB[CFG_RGB_LED_COUNT];
};
static PatternState s_pattern = {};

// ── Public API ────────────────────────────────────────────────────────────
void led_pca_init()
{
    memset(&s_pattern, 0, sizeof(s_pattern));
    s_pattern.type = PatternType::NONE;
    led_pca_off();
    LOG("[LED:PCA] Ready — " + String(CFG_RGB_LED_COUNT) + " RGB LEDs");
}

void led_pca_set(uint8_t id, uint8_t r, uint8_t g, uint8_t b)
{
    if (id >= CFG_RGB_LED_COUNT)
        return;
    s_r[id] = r;
    s_g[id] = g;
    s_b[id] = b;
    setLed(id, r, g, b);
}

void led_pca_all(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint8_t i = 0; i < CFG_RGB_LED_COUNT; i++)
        led_pca_set(i, r, g, b);
}

void led_pca_off()
{
    uint8_t off = CFG_RGB_COMMON_ANODE ? 255 : 0;
    led_pca_all(off, off, off);
}

// Non-blocking — replaces the old vTaskDelay loop.
// Progress is advanced every led_pca_handle() call.
void led_pca_fade(uint8_t id, uint8_t tr, uint8_t tg, uint8_t tb, uint32_t ms)
{
    if (id >= CFG_RGB_LED_COUNT)
        return;
    s_fade.active = true;
    s_fade.id = id;
    s_fade.fromR = s_r[id];
    s_fade.fromG = s_g[id];
    s_fade.fromB = s_b[id];
    s_fade.toR = tr;
    s_fade.toG = tg;
    s_fade.toB = tb;
    s_fade.startMs = millis();
    s_fade.durationMs = ms;
}

// ── Pattern API ───────────────────────────────────────────────────────────
void led_pca_pattern_stop()
{
    s_pattern.type = PatternType::NONE;
    s_fade.active = false;
    led_pca_off();
    LOG("[LED:PCA] Pattern stopped");
}

void led_pca_pattern_blink(uint8_t id, uint8_t r, uint8_t g, uint8_t b,
                           uint32_t on_ms, uint32_t off_ms)
{
    led_pca_pattern_stop();
    s_pattern = {};
    s_pattern.type = PatternType::BLINK;
    s_pattern.id = id;
    s_pattern.r = r;
    s_pattern.g = g;
    s_pattern.b = b;
    s_pattern.ms = on_ms;
    s_pattern.ms2 = off_ms;
    s_pattern.phase = false;
    s_pattern.nextTick = millis();
    LOG("[LED:PCA] Blink started");
}

void led_pca_pattern_pulse(uint8_t id, uint8_t r, uint8_t g, uint8_t b, uint32_t ms)
{
    led_pca_pattern_stop();
    s_pattern = {};
    s_pattern.type = PatternType::PULSE;
    s_pattern.id = id;
    s_pattern.r = r;
    s_pattern.g = g;
    s_pattern.b = b;
    s_pattern.ms = ms;
    s_pattern.fadingUp = true;
    s_pattern.fadeStart = millis();
    LOG("[LED:PCA] Pulse started");
}

void led_pca_pattern_chase(uint8_t r, uint8_t g, uint8_t b, uint32_t gap_ms)
{
    led_pca_pattern_stop();
    s_pattern = {};
    s_pattern.type = PatternType::CHASE;
    s_pattern.r = r;
    s_pattern.g = g;
    s_pattern.b = b;
    s_pattern.ms = gap_ms;
    s_pattern.chaseIdx = 0;
    s_pattern.nextTick = millis();
    LOG("[LED:PCA] Chase started");
}

// ── handle() — tick called from led_handle() every loop() iteration ───────
void led_pca_handle()
{
    uint32_t now = millis();

    // ── One-shot fade ─────────────────────────────────────────────────────
    if (s_fade.active)
    {
        // t >= 1.0 when duration==0 — snaps immediately, no division hazard
        float t = (s_fade.durationMs == 0) ? 1.0f
                                           : (float)(now - s_fade.startMs) / (float)s_fade.durationMs;
        if (t >= 1.0f)
        {
            led_pca_set(s_fade.id, s_fade.toR, s_fade.toG, s_fade.toB);
            s_fade.active = false;
        }
        else
        {
            // Sine ease-in matches the feel of the old blocking fade loop
            float curved = sinf(t * (float)M_PI * 0.5f);
            uint8_t r = (uint8_t)(s_fade.fromR + (int)(s_fade.toR - s_fade.fromR) * curved);
            uint8_t g = (uint8_t)(s_fade.fromG + (int)(s_fade.toG - s_fade.fromG) * curved);
            uint8_t b = (uint8_t)(s_fade.fromB + (int)(s_fade.toB - s_fade.fromB) * curved);
            led_pca_set(s_fade.id, r, g, b);
        }
    }

    if (s_pattern.type == PatternType::NONE)
        return;

    // ── Pattern tick ──────────────────────────────────────────────────────
    switch (s_pattern.type)
    {
    // ── BLINK ─────────────────────────────────────────────────────────────
    case PatternType::BLINK:
        if (now < s_pattern.nextTick)
            break;
        if (!s_pattern.phase)
        {
            led_pca_set(s_pattern.id, s_pattern.r, s_pattern.g, s_pattern.b);
            s_pattern.nextTick = now + s_pattern.ms;
        }
        else
        {
            led_pca_set(s_pattern.id, 0, 0, 0);
            s_pattern.nextTick = now + s_pattern.ms2;
        }
        s_pattern.phase = !s_pattern.phase;
        break;

    // ── PULSE ─────────────────────────────────────────────────────────────
    // Runs every handle() call; dirty check suppresses I²C writes when the
    // computed duty rounds to the same byte as the last write.
    case PatternType::PULSE:
    {
        if (s_pattern.ms == 0)
            break; // misconfigured — ignore
        float t = (float)(now - s_pattern.fadeStart) / (float)s_pattern.ms;
        if (t >= 1.0f)
        {
            s_pattern.fadingUp = !s_pattern.fadingUp;
            s_pattern.fadeStart = now;
            t = 0.0f;
        }
        float curved = sinf(t * (float)M_PI * 0.5f);
        float intensity = s_pattern.fadingUp ? curved : (1.0f - curved);
        uint8_t r = (uint8_t)(s_pattern.r * intensity);
        uint8_t g = (uint8_t)(s_pattern.g * intensity);
        uint8_t b = (uint8_t)(s_pattern.b * intensity);
        uint8_t id = s_pattern.id;
        if (r != s_pattern.lastR[id] ||
            g != s_pattern.lastG[id] ||
            b != s_pattern.lastB[id])
        {
            led_pca_set(id, r, g, b);
            s_pattern.lastR[id] = r;
            s_pattern.lastG[id] = g;
            s_pattern.lastB[id] = b;
        }
        break;
    }

    // ── CHASE ─────────────────────────────────────────────────────────────
    // Writes only the two LEDs that change per step (prev→off, current→on):
    // 6 I²C writes vs. the old led_pca_off() + led_pca_set() = 15 writes.
    case PatternType::CHASE:
        if (now < s_pattern.nextTick)
            break;
        {
            uint8_t prev = (s_pattern.chaseIdx == 0)
                               ? CFG_RGB_LED_COUNT - 1
                               : s_pattern.chaseIdx - 1;
            led_pca_set(prev, 0, 0, 0);
            led_pca_set(s_pattern.chaseIdx,
                        s_pattern.r, s_pattern.g, s_pattern.b);
            s_pattern.chaseIdx = (s_pattern.chaseIdx + 1) % CFG_RGB_LED_COUNT;
            s_pattern.nextTick = now + s_pattern.ms;
        }
        break;

    default:
        break;
    }
}

// ── Command handler ───────────────────────────────────────────────────────
void led_pca_cmd(const String &msg)
{
    auto tok = [&](int n) -> String
    {
        int pos = 0;
        for (int i = 0; i < n; i++)
        {
            pos = msg.indexOf(':', pos) + 1;
            if (pos == 0)
                return "";
        }
        int end = msg.indexOf(':', pos);
        return end == -1 ? msg.substring(pos) : msg.substring(pos, end);
    };

    String action = tok(0);

    if (action == "off")
    {
        led_pca_off();
        return;
    }

    if (action == "all")
    {
        led_pca_all(tok(1).toInt(), tok(2).toInt(), tok(3).toInt());
        return;
    }

    if (action == "set")
    {
        led_pca_set(tok(1).toInt(),
                    tok(2).toInt(), tok(3).toInt(), tok(4).toInt());
        return;
    }

    if (action == "fade")
    {
        led_pca_fade(tok(1).toInt(),
                     tok(2).toInt(), tok(3).toInt(), tok(4).toInt(),
                     tok(5).toInt());
        return;
    }

    if (action == "pattern")
    {
        String pat = tok(1);
        if (pat == "stop")
        {
            led_pca_pattern_stop();
            return;
        }
        if (pat == "blink")
        {
            led_pca_pattern_blink(tok(2).toInt(),
                                  tok(3).toInt(), tok(4).toInt(), tok(5).toInt(),
                                  tok(6).toInt(), tok(7).toInt());
            return;
        }
        if (pat == "pulse")
        {
            led_pca_pattern_pulse(tok(2).toInt(),
                                  tok(3).toInt(), tok(4).toInt(), tok(5).toInt(),
                                  tok(6).toInt());
            return;
        }
        if (pat == "chase")
        {
            led_pca_pattern_chase(tok(2).toInt(), tok(3).toInt(),
                                  tok(4).toInt(), tok(5).toInt());
            return;
        }
    }

    LOG("[LED:PCA] Unknown. Use: set | fade | all | off | pattern:blink/pulse/chase/stop");
}

#endif // FEATURE_RGB