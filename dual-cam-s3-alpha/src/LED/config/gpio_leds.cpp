#ifdef FEATURE_GPIO_LEDS

#include "gpio_leds.h"
#include "Build/config/target_config.h"
#include "Build/Log/logger.h"
#include "Build/Log/cmd_registry.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <string.h>

#ifdef LEDC_HIGH_SPEED_MODE
static constexpr ledc_mode_t kGpioLedcMode = LEDC_HIGH_SPEED_MODE;
#else
static constexpr ledc_mode_t kGpioLedcMode = LEDC_LOW_SPEED_MODE;
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Internal state
// ─────────────────────────────────────────────────────────────────────────────

// Candidate GPIO pins loaded from config — copied to RAM so we can iterate.
static const uint8_t k_candidates[] = CFG_GPIO_LED_PINS;
static const uint8_t k_candidateCount = sizeof(k_candidates);

struct GpioLedEntry
{
    uint8_t pin;
    uint8_t channel; // LEDC channel assigned to this pin
    bool active;     // in use (detected or force-enabled)
    uint8_t duty;    // last written duty (0-255)
};

static GpioLedEntry s_leds[CFG_GPIO_LED_MAX_PINS];
static uint8_t s_ledCount = 0;

static GpioLedMode s_mode = GpioLedMode::AUTO;
static uint32_t s_lastScan = 0;
static uint32_t s_breatheTick = 0;
static float s_breathePhase = 0.0f;

// ─────────────────────────────────────────────────────────────────────────────
// LEDC helpers
// ─────────────────────────────────────────────────────────────────────────────

static bool ledc_setup_pin(uint8_t channel, uint8_t pin)
{
    ledc_timer_config_t timer = {
        .speed_mode = kGpioLedcMode,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = (ledc_timer_t)CFG_GPIO_LED_LEDC_TIMER,
        .freq_hz = CFG_GPIO_LED_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&timer) != ESP_OK)
        return false;

    ledc_channel_config_t ch = {
        .gpio_num = pin,
        .speed_mode = kGpioLedcMode,
        .channel = (ledc_channel_t)channel,
        .timer_sel = (ledc_timer_t)CFG_GPIO_LED_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    return ledc_channel_config(&ch) == ESP_OK;
}

static void ledc_write_duty(uint8_t channel, uint8_t duty)
{
    ledc_set_duty(kGpioLedcMode, (ledc_channel_t)channel, duty);
    ledc_update_duty(kGpioLedcMode, (ledc_channel_t)channel);
}

// Old version used analogRead (ADC2 — broken when WiFi is active).
// New version: attempt to drive pin HIGH briefly; if the pin sinks current
// (LED-to-GND) it pulls back down noticeably. Falls back to always-true so
// candidate pins are always registered — use gpled:disable to remove strays.
static bool probe_pin(uint8_t pin)
{
    // Drive HIGH for a short pulse, then release and sample.
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    delayMicroseconds(200);

    pinMode(pin, INPUT); // release — no pull-up
    delayMicroseconds(300);

    int reading = digitalRead(pin);

    LOGF("[GPLED] probe pin %u: digital %d\n", pin, reading);

    // If pin reads LOW after releasing, something is pulling it down (LED to GND).
    // If you want to always register candidate pins unconditionally, just return true.
    return (reading == LOW);
}

// ─────────────────────────────────────────────────────────────────────────────
// Active-list management
// ─────────────────────────────────────────────────────────────────────────────

static GpioLedEntry *find_entry(uint8_t pin)
{
    for (uint8_t i = 0; i < s_ledCount; i++)
        if (s_leds[i].pin == pin)
            return &s_leds[i];
    return nullptr;
}

static GpioLedEntry *add_entry(uint8_t pin)
{
    if (s_ledCount >= CFG_GPIO_LED_MAX_PINS)
    {
        LOG("[GPLED] Pin table full — increase CFG_GPIO_LED_MAX_PINS");
        return nullptr;
    }
    uint8_t ch = CFG_GPIO_LED_START_CHANNEL + s_ledCount;
    if (!ledc_setup_pin(ch, pin))
    {
        LOGF("[GPLED] LEDC init failed for pin %u\n", pin);
        return nullptr;
    }
    s_leds[s_ledCount] = {pin, ch, true, 0};
    return &s_leds[s_ledCount++];
}

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

void gpio_leds_scan()
{
    LOG("[GPLED] Scanning candidate pins...");

    for (uint8_t i = 0; i < k_candidateCount; i++)
    {
        uint8_t pin = k_candidates[i];
        GpioLedEntry *e = find_entry(pin);

        bool detected = probe_pin(pin);

        if (detected)
        {
            if (e == nullptr)
            {
                e = add_entry(pin);
                if (e)
                    LOG("[GPLED] Detected LED on pin " + String(pin));
            }
            else
            {
                e->active = true;
            }
        }
        else
        {
            // Not detected — deactivate if previously active (but keep the
            // LEDC channel allocated; it might just be temporarily off).
            if (e && e->active)
            {
                e->active = false;
                ledc_write_duty(e->channel, 0);
                LOG("[GPLED] Pin " + String(pin) + " no longer detected — deactivated");
            }
        }
    }

    // Summary
    uint8_t activeCount = 0;
    for (uint8_t i = 0; i < s_ledCount; i++)
        if (s_leds[i].active)
            activeCount++;

    LOG("[GPLED] Scan done. Active pins: " + String(activeCount));
    s_lastScan = millis();
}

void gpio_leds_init()
{
    s_ledCount = 0;
    s_mode = GpioLedMode::MANUAL;
    s_breathePhase = 0.0f;
    s_lastScan = 0;
    s_breatheTick = 0;

    gpio_leds_scan();

    LOG("[GPLED] Init done — mode: MANUAL");
}

void gpio_leds_set_mode(GpioLedMode mode)
{
    s_mode = mode;
    const char *names[] = {"AUTO", "MANUAL", "PCA"};
    LOG("[GPLED] Mode -> " + String(names[(uint8_t)mode]));

    if (mode == GpioLedMode::PCA)
    {
        // Force all GPIO LEDs off immediately so PCA module has clean state.
        for (uint8_t i = 0; i < s_ledCount; i++)
            ledc_write_duty(s_leds[i].channel, 0);
    }
}

GpioLedMode gpio_leds_get_mode() { return s_mode; }

void gpio_leds_set(uint8_t pin, uint8_t duty)
{
    if (s_mode != GpioLedMode::MANUAL)
    {
        LOG("[GPLED] Ignored — not in MANUAL mode");
        return;
    }
    GpioLedEntry *e = find_entry(pin);
    if (!e || !e->active)
    {
        LOG("[GPLED] Pin " + String(pin) + " not active");
        return;
    }
    e->duty = duty;
    ledc_write_duty(e->channel, duty);
}

void gpio_leds_all(uint8_t duty)
{
    if (s_mode != GpioLedMode::MANUAL)
    {
        LOG("[GPLED] Ignored — not in MANUAL mode");
        return;
    }
    for (uint8_t i = 0; i < s_ledCount; i++)
    {
        if (!s_leds[i].active)
            continue;
        s_leds[i].duty = duty;
        ledc_write_duty(s_leds[i].channel, duty);
    }
}

void gpio_leds_enable(uint8_t pin)
{
    GpioLedEntry *e = find_entry(pin);
    if (e)
    {
        e->active = true;
        LOG("[GPLED] Pin " + String(pin) + " force-enabled");
    }
    else
    {
        e = add_entry(pin);
        if (e)
            LOG("[GPLED] Pin " + String(pin) + " added + enabled");
    }
}

void gpio_leds_disable(uint8_t pin)
{
    GpioLedEntry *e = find_entry(pin);
    if (!e)
    {
        LOG("[GPLED] Pin " + String(pin) + " not found");
        return;
    }
    e->active = false;
    ledc_write_duty(e->channel, 0);
    LOG("[GPLED] Pin " + String(pin) + " disabled");
}

// ─────────────────────────────────────────────────────────────────────────────
// handle() — called from loop(), drives AUTO breathe + periodic re-scan
// ─────────────────────────────────────────────────────────────────────────────
void gpio_leds_handle()
{
    if (s_mode == GpioLedMode::PCA)
        return;

    uint32_t now = millis();

    // Periodic re-scan (AUTO mode only — in MANUAL the user controls the list)
    if (s_mode == GpioLedMode::AUTO &&
        (now - s_lastScan) >= CFG_GPIO_LED_SCAN_INTERVAL_MS)
    {
        gpio_leds_scan();
    }

    if (s_mode != GpioLedMode::AUTO)
        return;

    // Breathe pattern — sine wave over all active pins in phase.
    if ((now - s_breatheTick) < CFG_GPIO_LED_BREATHE_STEP_MS)
        return;
    s_breatheTick = now;

    s_breathePhase += CFG_GPIO_LED_BREATHE_INC;
    if (s_breathePhase > TWO_PI)
        s_breathePhase -= TWO_PI;

    // sin goes -1..1 → remap to 0..255
    float s = (sinf(s_breathePhase) + 1.0f) * 0.5f;
    uint8_t duty = (uint8_t)(s * 255.0f);

    for (uint8_t i = 0; i < s_ledCount; i++)
    {
        if (!s_leds[i].active)
            continue;
        s_leds[i].duty = duty;
        ledc_write_duty(s_leds[i].channel, duty);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// MQTT command handler
// ─────────────────────────────────────────────────────────────────────────────
//
//  gpled:mode:auto | manual | pca
//  gpled:scan
//  gpled:set:<pin>:<duty>
//  gpled:all:<duty>
//  gpled:enable:<pin>
//  gpled:disable:<pin>
//  gpled:status

static void gpledCmdHandler(const String &msg)
{
    // Token helper: returns the Nth colon-delimited field (0-indexed)
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

    String action = tok(1); // field after "gpled"

    if (action == "mode")
    {
        String m = tok(2);
        if (m == "auto")
            gpio_leds_set_mode(GpioLedMode::AUTO);
        else if (m == "manual")
            gpio_leds_set_mode(GpioLedMode::MANUAL);
        else if (m == "pca")
            gpio_leds_set_mode(GpioLedMode::PCA);
        else
            LOG("[GPLED] Unknown mode: " + m + " — use auto / manual / pca");
        return;
    }

    if (action == "scan")
    {
        if (s_mode == GpioLedMode::PCA)
        {
            LOG("[GPLED] Can't scan in PCA mode — switch mode first");
            return;
        }
        gpio_leds_scan();
        return;
    }

    if (action == "set")
    {
        uint8_t pin = (uint8_t)tok(2).toInt();
        uint8_t duty = (uint8_t)constrain(tok(3).toInt(), 0, 255);
        gpio_leds_set(pin, duty);
        return;
    }

    if (action == "all")
    {
        uint8_t duty = (uint8_t)constrain(tok(2).toInt(), 0, 255);
        gpio_leds_all(duty);
        return;
    }

    if (action == "enable")
    {
        gpio_leds_enable((uint8_t)tok(2).toInt());
        return;
    }

    if (action == "disable")
    {
        gpio_leds_disable((uint8_t)tok(2).toInt());
        return;
    }

    if (action == "status")
    {
        const char *modeNames[] = {"AUTO", "MANUAL", "PCA"};
        String out = "[GPLED] Mode: " + String(modeNames[(uint8_t)s_mode]) +
                     "  Pins(" + String(s_ledCount) + "): ";
        for (uint8_t i = 0; i < s_ledCount; i++)
        {
            out += String(s_leds[i].pin);
            out += s_leds[i].active ? "(on) " : "(off) ";
        }
        LOG(out);
        return;
    }

    LOG("[GPLED] Unknown cmd. Use: mode | scan | set | all | enable | disable | status");
}

void gpio_leds_register_commands()
{
    cmd_register("gpled:", gpledCmdHandler);
}

#endif // FEATURE_GPIO_LEDS
