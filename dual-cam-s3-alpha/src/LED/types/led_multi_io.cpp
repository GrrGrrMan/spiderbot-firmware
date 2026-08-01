#ifdef FEATURE_GPIO_LEDS

#include "led_multi_io.h"
#include "Build/config/target_config.h"
#include "Build/Log/logger.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <string.h>

// ─────────────────────────────────────────────────────────────────────────────
// led_multi_io.cpp — Multi-LED control on bare GPIO pins via LEDC PWM.
// ─────────────────────────────────────────────────────────────────────────────

#ifdef LEDC_HIGH_SPEED_MODE
static constexpr ledc_mode_t kLedcMode = LEDC_HIGH_SPEED_MODE;
#else
static constexpr ledc_mode_t kLedcMode = LEDC_LOW_SPEED_MODE;
#endif

// ── Internal state ────────────────────────────────────────────────────────
static const uint8_t k_candidates[]  = CFG_GPIO_LED_PINS;
static const uint8_t k_candidateCount = sizeof(k_candidates);

struct GpioLedEntry
{
    uint8_t pin;
    uint8_t channel;
    bool active;
    uint8_t duty;
    // Soft fade state
    bool fading;
    uint8_t fadeFrom;
    uint8_t fadeTo;
    uint32_t fadeStart;
    uint32_t fadeDuration;
};

static GpioLedEntry  s_leds[CFG_GPIO_LED_MAX_PINS];
static uint8_t       s_ledCount    = 0;
static LedMultiMode  s_mode        = LedMultiMode::AUTO;
static uint32_t      s_lastScan    = 0;
static uint32_t      s_breatheTick = 0;
static float         s_breathePhase = 0.0f;

// ── LEDC helpers ──────────────────────────────────────────────────────────
static bool ledc_setup_pin(uint8_t channel, uint8_t pin)
{
    ledc_timer_config_t timer = {
        .speed_mode      = kLedcMode,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num       = (ledc_timer_t)CFG_GPIO_LED_LEDC_TIMER,
        .freq_hz         = CFG_GPIO_LED_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&timer) != ESP_OK) return false;

    ledc_channel_config_t ch = {
        .gpio_num   = pin,
        .speed_mode = kLedcMode,
        .channel    = (ledc_channel_t)channel,
        .timer_sel  = (ledc_timer_t)CFG_GPIO_LED_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    return ledc_channel_config(&ch) == ESP_OK;
}

static void ledc_write_duty(uint8_t channel, uint8_t duty)
{
    ledc_set_duty(kLedcMode, (ledc_channel_t)channel, duty);
    ledc_update_duty(kLedcMode, (ledc_channel_t)channel);
}

// ── Pin detection ─────────────────────────────────────────────────────────
// Circuit assumption: LED anode → VCC (via resistor), cathode → GPIO pin.
// When floating, the LED forward-biases and pulls the pin toward VCC → ADC
// reads high.  We threshold at CFG_GPIO_LED_DETECT_MV.
// removed — analogRead on ADC2 pins (e.g. GPIO13) is unreliable
// Instead use led:multi:disable:<pin> to remove strays.
static bool probe_pin(uint8_t pin)
{
    pinMode(pin, INPUT);
    delayMicroseconds(500);
    int raw = analogRead(pin);
    int mv  = (raw * 3300) / 4095;
    LOGF("[LED:MULTI] probe pin %u: %d mV (raw %d)\n", pin, mv, raw);
    return mv >= CFG_GPIO_LED_DETECT_MV;
}

// ── Active-list management ────────────────────────────────────────────────
static GpioLedEntry *find_entry(uint8_t pin)
{
    for (uint8_t i = 0; i < s_ledCount; i++)
        if (s_leds[i].pin == pin) return &s_leds[i];
    return nullptr;
}

static GpioLedEntry *add_entry(uint8_t pin)
{
    if (s_ledCount >= CFG_GPIO_LED_MAX_PINS)
    {
        LOG("[LED:MULTI] Pin table full — increase CFG_GPIO_LED_MAX_PINS");
        return nullptr;
    }
    uint8_t ch = CFG_GPIO_LED_START_CHANNEL + s_ledCount;
    if (!ledc_setup_pin(ch, pin))
    {
        LOGF("[LED:MULTI] LEDC init failed for pin %u\n", pin);
        return nullptr;
    }
    s_leds[s_ledCount] = {pin, ch, true, 0, false, 0, 0, 0, 0};
    return &s_leds[s_ledCount++];
}

// ── Public API ────────────────────────────────────────────────────────────
void led_multi_scan() // dum filter needed due to ADC limitations (do not change until new board or adapter is bought)
{
    LOG("[LED:MULTI] Loading configured pins...");
    for (uint8_t i = 0; i < k_candidateCount; i++)
    {
        uint8_t pin = k_candidates[i];
        GpioLedEntry *e = find_entry(pin);

        if (e == nullptr)
        {
            e = add_entry(pin);
            if (e)
                LOG("[LED:MULTI] Activated configured pin " + String(pin));
        }
        else
        {
            e->active = true;
        }
    }
    s_lastScan = millis();
}

void led_multi_init()
{
    s_ledCount     = 0;
    s_mode = LedMultiMode::MANUAL;
    s_breathePhase = 0.0f;
    s_lastScan     = 0;
    s_breatheTick  = 0;
    led_multi_scan();
}

void led_multi_set_mode(LedMultiMode mode)
{
    s_mode = mode;
    const char *names[] = {"AUTO", "MANUAL", "PCA"};
    LOG("[LED:MULTI] Mode → " + String(names[(uint8_t)mode]));

    if (mode == LedMultiMode::PCA)
    {
        // Force all GPIO LEDs off so PCA module has clean state.
        for (uint8_t i = 0; i < s_ledCount; i++)
            ledc_write_duty(s_leds[i].channel, 0);
    }
}

LedMultiMode led_multi_get_mode() { return s_mode; }

void led_multi_set(uint8_t pin, uint8_t duty)
{
    if (s_mode != LedMultiMode::MANUAL) { LOG("[LED:MULTI] Ignored — not in MANUAL mode"); return; }
    GpioLedEntry *e = find_entry(pin);
    if (!e || !e->active) { LOG("[LED:MULTI] Pin " + String(pin) + " not active"); return; }
    e->duty = duty;
    ledc_write_duty(e->channel, duty);
}

void led_multi_all(uint8_t duty)
{
    if (s_mode != LedMultiMode::MANUAL) { LOG("[LED:MULTI] Ignored — not in MANUAL mode"); return; }
    for (uint8_t i = 0; i < s_ledCount; i++)
    {
        if (!s_leds[i].active) continue;
        s_leds[i].duty = duty;
        ledc_write_duty(s_leds[i].channel, duty);
    }
}

void led_multi_enable(uint8_t pin)
{
    GpioLedEntry *e = find_entry(pin);
    if (e)
    {
        e->active = true;
        LOG("[LED:MULTI] Pin " + String(pin) + " force-enabled");
    }
    else
    {
        e = add_entry(pin);
        if (e) LOG("[LED:MULTI] Pin " + String(pin) + " added + enabled");
    }
}

void led_multi_disable(uint8_t pin)
{
    GpioLedEntry *e = find_entry(pin);
    if (!e) { LOG("[LED:MULTI] Pin " + String(pin) + " not found"); return; }
    e->active = false;
    ledc_write_duty(e->channel, 0);
    LOG("[LED:MULTI] Pin " + String(pin) + " disabled");
}

void led_multi_fade(uint8_t pin, uint8_t target, uint32_t ms)
{
    if (s_mode != LedMultiMode::MANUAL)
    {
        LOG("[LED:MULTI] Ignored — not in MANUAL mode");
        return;
    }
    GpioLedEntry *e = find_entry(pin);
    if (!e || !e->active)
    {
        LOG("[LED:MULTI] Pin " + String(pin) + " not active");
        return;
    }
    e->fadeFrom = e->duty;
    e->fadeTo = target;
    e->fadeStart = millis();
    e->fadeDuration = ms;
    e->fading = true;
}

void led_multi_fade_all(uint8_t target, uint32_t ms)
{
    if (s_mode != LedMultiMode::MANUAL)
    {
        LOG("[LED:MULTI] Ignored — not in MANUAL mode");
        return;
    }
    for (uint8_t i = 0; i < s_ledCount; i++)
        if (s_leds[i].active)
            led_multi_fade(s_leds[i].pin, target, ms);
}

// ── Sequence task ─────────────────────────────────────────────────────────
// Self-contained FreeRTOS task inspired by led_single_io loop + WLED playlist.
// Each step runs its own sine-eased PWM timing loop at ~50 Hz.
// Writes ledc directly — no contention with handle() fade tick.

#define LED_MULTI_SEQ_TICK_MS 20 // 50 Hz update rate

static TaskHandle_t s_seqTask = nullptr;
static LedMultiSeqStep s_seqSteps[LED_MULTI_SEQ_MAX_STEPS];
static uint8_t s_seqCount = 0;

static void seqApplyStep(const LedMultiSeqStep &step)
{
    GpioLedEntry *e = find_entry(step.pin);
    if (!e || !e->active)
        return;

    uint8_t from = e->duty;
    uint32_t start = millis();

    if (step.fadems == 0)
    {
        e->duty = step.duty;
        ledc_write_duty(e->channel, step.duty);
    }
    else
    {
        for (;;)
        {
            uint32_t elapsed = millis() - start;
            if (elapsed >= step.fadems)
            {
                e->duty = step.duty;
                ledc_write_duty(e->channel, step.duty);
                break;
            }
            float t = (float)elapsed / (float)step.fadems;
            float curved = sinf(t * (float)M_PI * 0.5f); // sine ease-in
            int16_t delta = (int16_t)step.duty - (int16_t)from;
            uint8_t duty = (uint8_t)(from + (int16_t)(delta * curved));
            if (duty != e->duty)
            {
                e->duty = duty;
                ledc_write_duty(e->channel, duty);
            }
            vTaskDelay(pdMS_TO_TICKS(LED_MULTI_SEQ_TICK_MS));
        }
    }

    if (step.holdms > 0)
        vTaskDelay(pdMS_TO_TICKS(step.holdms));
}

static void seqTaskFn(void *)
{
    for (;;)
        for (uint8_t i = 0; i < s_seqCount; i++)
            seqApplyStep(s_seqSteps[i]);
}

void led_multi_seq_start(const LedMultiSeqStep *steps, uint8_t count)
{
    led_multi_seq_stop();
    s_seqCount = count > LED_MULTI_SEQ_MAX_STEPS ? LED_MULTI_SEQ_MAX_STEPS : count;
    for (uint8_t i = 0; i < s_seqCount; i++)
        s_seqSteps[i] = steps[i];
    xTaskCreate(seqTaskFn, "led_seq", 2048, nullptr, 1, &s_seqTask);
    LOG("[LED:MULTI] Seq started — " + String(s_seqCount) + " step(s)");
}

void led_multi_seq_stop()
{
    if (s_seqTask)
    {
        vTaskDelete(s_seqTask);
        s_seqTask = nullptr;
        LOG("[LED:MULTI] Seq stopped");
    }
}

bool led_multi_seq_active() { return s_seqTask != nullptr; }

// ── handle() — called from led_handle() / loop() ─────────────────────────
void led_multi_handle()
{
    if (s_mode == LedMultiMode::PCA) return;

    uint32_t now = millis();

    // ── Soft fade tick ────────────────────────────────────────────────────
    for (uint8_t i = 0; i < s_ledCount; i++)
    {
        GpioLedEntry &e = s_leds[i];
        if (!e.active || !e.fading)
            continue;

        float t = (float)(now - e.fadeStart);
        float T = (float)e.fadeDuration;

        if (t >= T)
        {
            e.duty = e.fadeTo;
            e.fading = false;
        }
        else
        {
            // Sine curve — same feel as led_single_io
            float p = sinf((t / T) * (float)M_PI * 0.5f);
            int16_t delta = (int16_t)e.fadeTo - (int16_t)e.fadeFrom;
            e.duty = (uint8_t)(e.fadeFrom + (int16_t)(delta * p));
        }
        ledc_write_duty(e.channel, e.duty);
    }

    if (s_mode == LedMultiMode::AUTO &&
        (now - s_lastScan) >= CFG_GPIO_LED_SCAN_INTERVAL_MS)
    {
        led_multi_scan();
    }

    if (s_mode != LedMultiMode::AUTO) return;

    // Breathe pattern — sine wave over all active pins in phase.
    if ((now - s_breatheTick) < CFG_GPIO_LED_BREATHE_STEP_MS) return;
    s_breatheTick = now;

    s_breathePhase += CFG_GPIO_LED_BREATHE_INC;
    if (s_breathePhase > TWO_PI) s_breathePhase -= TWO_PI;

    float   s    = (sinf(s_breathePhase) + 1.0f) * 0.5f;
    uint8_t duty = (uint8_t)(s * 255.0f);

    for (uint8_t i = 0; i < s_ledCount; i++)
    {
        if (!s_leds[i].active) continue;
        s_leds[i].duty = duty;
        ledc_write_duty(s_leds[i].channel, duty);
    }
}

// ── Command handler ───────────────────────────────────────────────────────
// msg = everything after "led:multi:" — e.g. "mode:auto" | "scan" | "set:12:128"
void led_multi_cmd(const String &msg)
{
    // Token helper: field N from colon-delimited msg (0-indexed from start)
    auto tok = [&](int n) -> String
    {
        int pos = 0;
        for (int i = 0; i < n; i++)
        {
            pos = msg.indexOf(':', pos) + 1;
            if (pos == 0) return "";
        }
        int end = msg.indexOf(':', pos);
        return end == -1 ? msg.substring(pos) : msg.substring(pos, end);
    };

    String action = tok(0);

    if (action == "mode")
    {
        String m = tok(1);
        if      (m == "auto")   led_multi_set_mode(LedMultiMode::AUTO);
        else if (m == "manual") led_multi_set_mode(LedMultiMode::MANUAL);
        else if (m == "pca")    led_multi_set_mode(LedMultiMode::PCA);
        else LOG("[LED:MULTI] Unknown mode: " + m + " — use auto / manual / pca");
        return;
    }

    if (action == "scan")
    {
        if (s_mode == LedMultiMode::PCA)
        {
            LOG("[LED:MULTI] Can't scan in PCA mode — switch mode first");
            return;
        }
        led_multi_scan();
        return;
    }

    if (action == "set")
    {
        uint8_t pin  = (uint8_t)tok(1).toInt();
        uint8_t duty = (uint8_t)constrain(tok(2).toInt(), 0, 255);
        led_multi_set(pin, duty);
        return;
    }

    if (action == "all")
    {
        uint8_t duty = (uint8_t)constrain(tok(1).toInt(), 0, 255);
        led_multi_all(duty);
        return;
    }

    if (action == "fade")
    {
        uint8_t pin = (uint8_t)tok(1).toInt();
        uint8_t target = (uint8_t)constrain(tok(2).toInt(), 0, 255);
        uint32_t ms = (uint32_t)tok(3).toInt();
        led_multi_fade(pin, target, ms);
        return;
    }

    if (action == "fadeall")
    {
        uint8_t target = (uint8_t)constrain(tok(1).toInt(), 0, 255);
        uint32_t ms = (uint32_t)tok(2).toInt();
        led_multi_fade_all(target, ms);
        return;
    }

    if (action == "seq")
    {
        String sub = tok(1);

        if (sub == "stop")
        {
            led_multi_seq_stop();
            return;
        }

        if (sub.startsWith("start"))
        {
            // Format: seq:start|pin:duty:fadems:holdms|pin:duty:fadems:holdms|...
            // holdms is optional — defaults to 0
            int pipePos = msg.indexOf('|');
            if (pipePos == -1)
            {
                LOG("[LED:MULTI] seq:start — no steps found. Format: seq:start|pin:duty:fadems:holdms|...");
                return;
            }

            String rest = msg.substring(pipePos + 1);
            LedMultiSeqStep steps[LED_MULTI_SEQ_MAX_STEPS];
            uint8_t count = 0;

            while (rest.length() > 0 && count < LED_MULTI_SEQ_MAX_STEPS)
            {
                int next = rest.indexOf('|');
                String seg = (next == -1) ? rest : rest.substring(0, next);
                rest = (next == -1) ? "" : rest.substring(next + 1);
                seg.trim();

                int c1 = seg.indexOf(':');
                int c2 = c1 == -1 ? -1 : seg.indexOf(':', c1 + 1);
                int c3 = c2 == -1 ? -1 : seg.indexOf(':', c2 + 1);
                if (c1 == -1 || c2 == -1)
                    continue;

                steps[count].pin = (uint8_t)seg.substring(0, c1).toInt();
                steps[count].duty = (uint8_t)constrain(seg.substring(c1 + 1, c2).toInt(), 0, 255);
                steps[count].fadems = (uint32_t)seg.substring(c2 + 1, c3 == -1 ? seg.length() : c3).toInt();
                steps[count].holdms = (c3 == -1) ? 0 : (uint32_t)seg.substring(c3 + 1).toInt();
                count++;
            }

            if (count == 0)
            {
                LOG("[LED:MULTI] seq:start — no valid steps parsed");
                return;
            }
            led_multi_seq_start(steps, count);
            return;
        }

        LOG("[LED:MULTI] Usage: seq:start|pin:duty:fadems[:holdms]|...  or  seq:stop");
        return;
    }

    if (action == "enable")
    {
        led_multi_enable((uint8_t)tok(1).toInt());
        return;
    }

    if (action == "disable")
    {
        led_multi_disable((uint8_t)tok(1).toInt());
        return;
    }

    if (action == "status")
    {
        const char *modeNames[] = {"AUTO", "MANUAL", "PCA"};
        String out = "[LED:MULTI] Mode: " + String(modeNames[(uint8_t)s_mode]) +
                     "  Pins(" + String(s_ledCount) + "): ";
        for (uint8_t i = 0; i < s_ledCount; i++)
        {
            out += String(s_leds[i].pin);
            out += s_leds[i].active ? "(on) " : "(off) ";
        }
        LOG(out);
        return;
    }

    LOG("[LED:MULTI] Unknown cmd. Use: mode | scan | set | all | enable | disable | status");
}

#endif // FEATURE_GPIO_LEDS
