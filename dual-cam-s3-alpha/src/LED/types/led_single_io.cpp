#ifdef FEATURE_LED

#include "led_single_io.h"
#include "Build/config/target_config.h"
#include "Build/Log/logger.h"
#include "driver/ledc.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
// led_single_io.cpp — Single onboard LED via LEDC PWM.
// ─────────────────────────────────────────────────────────────────────────────

#ifdef LEDC_HIGH_SPEED_MODE
static constexpr ledc_mode_t kLedcMode = LEDC_HIGH_SPEED_MODE;
#else
static constexpr ledc_mode_t kLedcMode = LEDC_LOW_SPEED_MODE;
#endif

// ── State ─────────────────────────────────────────────────────────────────
static uint8_t      s_duty     = 0;
static TaskHandle_t s_loopTask = nullptr;

static struct
{
    uint32_t gap_ms;
    uint8_t  count;
    char     cmds[CFG_LED_LOOP_MAX_CMDS][CFG_LED_LOOP_CMD_LEN];
} s_loopDef;



struct SoftFadeState {
    bool     active      = false;
    uint8_t  from, to;
    uint32_t startMs, durationMs;
    LedFadeCurve curve;
};
static SoftFadeState s_fade;

struct TimerEntry { uint32_t fireAt; uint8_t duty; bool active; };
static TimerEntry s_timers[CFG_LED_TIMER_MAX];

// ── Curve math — must live ABOVE led_single_handle() ─────────────────────
static float applyCurve(float p, LedFadeCurve curve)
{
    switch (curve)
    {
    case LedFadeCurve::SINE:        return sinf(p * M_PI * 0.5f);
    case LedFadeCurve::EXPONENTIAL: return (expf(p) - 1.0f) / (M_E - 1.0f);
    case LedFadeCurve::LOGARITHMIC: return logf(p * (M_E - 1.0f) + 1.0f);
    case LedFadeCurve::QUADRATIC:   return p * p;
    case LedFadeCurve::INV_QUAD:    return 1.0f - (1.0f - p) * (1.0f - p);
    default:                        return p;  // LINEAR
    }
}

void led_single_handle()
{
    uint32_t now = millis();

    for (auto &t : s_timers)
    {
        if (t.active && now >= t.fireAt)
        {
            t.active = false;
            led_single_set(t.duty);
        }
    }

    if (!s_fade.active) return;

    float t = (float)(now - s_fade.startMs);
    float T = (float)s_fade.durationMs;

    if (t >= T)
    {
        led_single_set(s_fade.to);
        s_fade.active = false;
        return;
    }

    float   progress = t / T;
    float   curved   = applyCurve(progress, s_fade.curve);
    // Guard against the fade going backwards if from > to
    int16_t delta    = (int16_t)s_fade.to - (int16_t)s_fade.from;
    uint8_t duty     = (uint8_t)(s_fade.from + (int16_t)(delta * curved));
    led_single_set(duty);
}

// ── Core helpers ──────────────────────────────────────────────────────────
void led_single_init()
{
    ledc_timer_config_t timer = {
        .speed_mode      = kLedcMode,
        .duty_resolution = (ledc_timer_bit_t)CFG_LED_RESOLUTION,
        .timer_num       = (ledc_timer_t)CFG_LED_TIMER,
        .freq_hz         = CFG_LED_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {
        .gpio_num   = CFG_LED_PIN,
        .speed_mode = kLedcMode,
        .channel    = (ledc_channel_t)CFG_LED_CHANNEL,
        .timer_sel  = (ledc_timer_t)CFG_LED_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&ch);

    ledc_fade_func_install(0);
    LOG("[LED:SINGLE] Ready on pin " + String(CFG_LED_PIN));
}

void led_single_set(uint8_t duty)
{
    s_duty = duty;
    ledc_set_duty(kLedcMode, (ledc_channel_t)CFG_LED_CHANNEL, duty);
    ledc_update_duty(kLedcMode, (ledc_channel_t)CFG_LED_CHANNEL);
}

void led_single_fade_to(uint8_t target, uint32_t ms)
{
    s_duty = target;
    ledc_set_fade_with_time(kLedcMode,
                            (ledc_channel_t)CFG_LED_CHANNEL, target, (int)ms);
    ledc_fade_start(kLedcMode,
                    (ledc_channel_t)CFG_LED_CHANNEL, LEDC_FADE_NO_WAIT);
}

uint8_t led_single_get() { return s_duty; }

// ── Loop task ─────────────────────────────────────────────────────────────
static void runCmd(const char *cmd)
{
    String s(cmd);

    if (s.startsWith("set:"))
    {
        uint8_t duty = (uint8_t)constrain(s.substring(4).toInt(), 0, 255);
        led_single_set(duty);
    }
    else if (s.startsWith("fade:"))
    {
        int colon2 = s.indexOf(':', 5);
        if (colon2 == -1) return;
        uint8_t  target = (uint8_t)constrain(s.substring(5, colon2).toInt(), 0, 255);
        uint32_t ms     = (uint32_t)s.substring(colon2 + 1).toInt();
        led_single_fade_to(target, ms);
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
    else if (s == "stop")
    {
        led_single_fade_to(0, CFG_LED_STOP_FADE_MS);
        vTaskDelay(pdMS_TO_TICKS(CFG_LED_STOP_FADE_MS));
    }
}

static void loopTaskFn(void *)
{
    for (;;)
    {
        for (uint8_t i = 0; i < s_loopDef.count; i++)
        {
            runCmd(s_loopDef.cmds[i]);
            vTaskDelay(pdMS_TO_TICKS(s_loopDef.gap_ms));
        }
    }
}

void led_single_loop_start(uint32_t gap_ms, const char *cmds[], uint8_t count)
{
    led_single_loop_stop();

    s_loopDef.gap_ms = gap_ms;
    s_loopDef.count  = (count > CFG_LED_LOOP_MAX_CMDS) ? CFG_LED_LOOP_MAX_CMDS : count;

    for (uint8_t i = 0; i < s_loopDef.count; i++)
    {
        strncpy(s_loopDef.cmds[i], cmds[i], CFG_LED_LOOP_CMD_LEN - 1);
        s_loopDef.cmds[i][CFG_LED_LOOP_CMD_LEN - 1] = '\0';
    }

    xTaskCreate(loopTaskFn, "led_loop", 2048, nullptr, 1, &s_loopTask);
    LOGF("[LED:SINGLE] Loop started — %d cmd(s), %ums gap\n",
         s_loopDef.count, (unsigned)gap_ms);
}

void led_single_loop_stop()
{
    if (s_loopTask)
    {
        vTaskDelete(s_loopTask);
        s_loopTask = nullptr;
        led_single_set(0);
        LOG("[LED:SINGLE] Loop stopped");
    }
}

bool led_single_loop_active() { return s_loopTask != nullptr; }

// ── Command handler ───────────────────────────────────────────────────────
// msg = everything after "led:single:" — e.g. "set:255" | "fade:128:500"
//       | "loop:start:250|set:255|set:0" | "loop:stop"
void led_single_cmd(const String &msg)
{
    int first  = msg.indexOf(':');
    int second = (first == -1) ? -1 : msg.indexOf(':', first + 1);

    String action = (first == -1) ? msg : msg.substring(0, first);

    if (action == "set")
    {
        uint8_t duty = (uint8_t)constrain(msg.substring(first + 1).toInt(), 0, 255);
        led_single_set(duty);
        LOG("[LED:SINGLE] set " + String(duty));
    }
    else if (action == "fade" && second != -1)
    {
        uint8_t  target = (uint8_t)constrain(msg.substring(first + 1, second).toInt(), 0, 255);
        uint32_t ms     = (uint32_t)msg.substring(second + 1).toInt();
        led_single_fade_to(target, ms);
        LOG("[LED:SINGLE] fade → " + String(target) + " over " + String(ms) + "ms");
    }
    else if (action == "loop")
    {
        // rest = "start:250|set:255|fade:0:300|..." or "stop"
        String rest   = msg.substring(first + 1);
        int    subEnd = rest.indexOf(':');
        String sub    = (subEnd == -1) ? rest : rest.substring(0, subEnd);

        if (sub == "stop")
        {
            led_single_loop_stop();
        }
        else if (sub == "start" && subEnd != -1)
        {
            String payload = rest.substring(subEnd + 1); // "250|set:255|set:0"
            int    pipePos = payload.indexOf('|');

            if (pipePos == -1)
            {
                LOG("[LED:SINGLE] loop:start missing commands — need gap|cmd1|cmd2");
                return;
            }

            uint32_t gap_ms = (uint32_t)payload.substring(0, pipePos).toInt();
            payload = payload.substring(pipePos + 1);

            const char *cmds[CFG_LED_LOOP_MAX_CMDS];
            String      cmdStrs[CFG_LED_LOOP_MAX_CMDS];
            uint8_t     count = 0;

            while (payload.length() > 0 && count < CFG_LED_LOOP_MAX_CMDS)
            {
                int next      = payload.indexOf('|');
                cmdStrs[count] = (next == -1) ? payload : payload.substring(0, next);
                payload        = (next == -1) ? "" : payload.substring(next + 1);
                cmdStrs[count].trim();
                cmds[count]    = cmdStrs[count].c_str();
                count++;
            }

            led_single_loop_start(gap_ms, cmds, count);
        }
        else
        {
            LOG("[LED:SINGLE] Usage: led:single:loop:start:<gap>|cmd1|cmd2  or  led:single:loop:stop");
        }
    }
    else
    {
        LOG("[LED:SINGLE] Unknown. Use: set | fade | loop:start | loop:stop");
    }
}

#endif // FEATURE_LED
