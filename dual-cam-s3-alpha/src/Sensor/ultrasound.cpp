#ifdef FEATURE_ULTRASOUND

#include "ultrasound.h"
#include "Build/config/target_config.h"
#include "Build/Log/logger.h"
#include "Build/Log/cmd_registry.h"
#include "Network/mqtt/mqtt_trigger.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Conditionally pull in led_multi so this compiles cleanly even without
// FEATURE_GPIO_LEDS, though link functionality will be disabled.
#ifdef FEATURE_GPIO_LEDS
#include "LED/types/led_multi_io.h"
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Shared state — written by task (core 1), read by handle() (core 0).
// All reads/writes of primitives ≤ 4 bytes are atomic on ESP32 Xtensa.
// ─────────────────────────────────────────────────────────────────────────────
static volatile float s_distanceCm = -1.0f; // <0 = out of range
static volatile uint8_t s_lastDuty = 0;
static volatile bool s_newReading = false;
static TaskHandle_t s_task = nullptr;

// ── Runtime config (main-thread only after init) ──────────────────────────
static uint8_t s_trigPin = CFG_ULTRASOUND_TRIG_PIN;
static uint8_t s_echoPin = CFG_ULTRASOUND_ECHO_PIN;
static uint8_t s_ledPin = CFG_ULTRASOUND_LED_PIN;
static float s_minCm = (float)CFG_ULTRASOUND_MIN_CM;
static float s_maxCm = (float)CFG_ULTRASOUND_MAX_CM;
static uint32_t s_sampleMs = CFG_ULTRASOUND_SAMPLE_MS;
static bool s_ledLinked = false;
static bool s_invertRatio = false;
static bool s_loggingOn = false;
static uint32_t s_logIntervalMs = 500;
static float s_thresholdCm = 0.0f; // 0 = threshold disabled
static uint32_t s_lastLog = 0;
static uint32_t s_lastStatus = 0;

// Health counters. Timeout uses the configured pulseIn timeout, so a failed
// read cannot block forever. Echo level after timeout helps distinguish
// stuck-HIGH wiring from normal no-echo/stuck-LOW cases.
static volatile uint32_t s_readCount = 0;
static volatile uint32_t s_timeoutCount = 0;
static volatile uint32_t s_stuckHighCount = 0;
static volatile uint32_t s_stuckLowCount = 0;
static volatile uint32_t s_impossibleCount = 0;
static volatile uint32_t s_validCount = 0;
static volatile float s_minSeenCm = 9999.0f;
static volatile float s_maxSeenCm = 0.0f;
static volatile float s_sumSeenCm = 0.0f;

// ─────────────────────────────────────────────────────────────────────────────
// Distance → duty mapping
// Default: near (minCm) = 255, far (maxCm) = 0. Invertible.
// ─────────────────────────────────────────────────────────────────────────────
static uint8_t distanceToDuty(float cm)
{
    float clamped = constrain(cm, s_minCm, s_maxCm);
    float ratio = (clamped - s_minCm) / (s_maxCm - s_minCm); // 0.0 near, 1.0 far
    if (!s_invertRatio)
        ratio = 1.0f - ratio; // near = bright (default)
    return (uint8_t)(ratio * 255.0f);
}

static float recordMeasurement(long duration)
{
    s_readCount++;

    if (duration == 0)
    {
        s_timeoutCount++;
        if (digitalRead(s_echoPin) == HIGH)
            s_stuckHighCount++;
        else
            s_stuckLowCount++;
        s_distanceCm = -1.0f;
        s_newReading = true;
        return -1.0f;
    }

    float cm = duration / 58.0f;
    if (cm < 1.0f || cm > 450.0f)
        s_impossibleCount++;
    else
    {
        s_validCount++;
        if (cm < s_minSeenCm)
            s_minSeenCm = cm;
        if (cm > s_maxSeenCm)
            s_maxSeenCm = cm;
        s_sumSeenCm += cm;
    }

    s_distanceCm = cm;
    s_newReading = true;
    return cm;
}

static void resetHealth()
{
    s_readCount = 0;
    s_timeoutCount = 0;
    s_stuckHighCount = 0;
    s_stuckLowCount = 0;
    s_impossibleCount = 0;
    s_validCount = 0;
    s_minSeenCm = 9999.0f;
    s_maxSeenCm = 0.0f;
    s_sumSeenCm = 0.0f;
}

static void publishHealth()
{
    uint32_t valid = s_validCount;
    float avg = valid ? (s_sumSeenCm / (float)valid) : -1.0f;
    float minCm = valid ? s_minSeenCm : -1.0f;
    float maxCm = valid ? s_maxSeenCm : -1.0f;

    String payload = "{\"reads\":" + String(s_readCount) +
                     ",\"valid\":" + String(valid) +
                     ",\"timeouts\":" + String(s_timeoutCount) +
                     ",\"stuck_high\":" + String(s_stuckHighCount) +
                     ",\"stuck_low\":" + String(s_stuckLowCount) +
                     ",\"impossible\":" + String(s_impossibleCount) +
                     ",\"min\":" + String(minCm, 1) +
                     ",\"avg\":" + String(avg, 1) +
                     ",\"max\":" + String(maxCm, 1) +
                     ",\"running\":" + String(s_task ? 1 : 0) + "}";
    mqtt_publish(CFG_ULTRASOUND_TOPIC_STATUS, payload);
    LOG("[SENSOR] Health " + payload);
}

// ─────────────────────────────────────────────────────────────────────────────
// FreeRTOS measurement task — pinned to core 1, away from WiFi stack
// ─────────────────────────────────────────────────────────────────────────────
static void ultrasonicTask(void *)
{
    // Cache pins locally — safe: restarts task on pin change
    const uint8_t trig = s_trigPin;
    const uint8_t echo = s_echoPin;

    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    digitalWrite(trig, LOW);
    delayMicroseconds(5); // settle

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(s_sampleMs));

        // ── Trigger pulse ─────────────────────────────────────────────────
        digitalWrite(trig, LOW);
        delayMicroseconds(2);
        digitalWrite(trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig, LOW);

        // ── Measure echo — pulseIn blocks this task, not core 0 ──────────
        long duration = pulseIn(echo, HIGH, CFG_ULTRASOUND_TIMEOUT_US);

        // duration == 0 → timeout (out of range, no object, or echo fault)
        recordMeasurement(duration);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal start / stop helpers (main thread only)
// ─────────────────────────────────────────────────────────────────────────────
static void startTask()
{
    if (s_task)
        return;
    s_newReading = false;
    xTaskCreatePinnedToCore(
        ultrasonicTask, "ultrasound",
        2048, nullptr, 1, &s_task, 1);
    LOG("[SENSOR] Started — trig:" + String(s_trigPin) +
        " echo:" + String(s_echoPin) +
        " every " + String(s_sampleMs) + "ms");
}

static void stopTask()
{
    if (!s_task)
        return;
    vTaskDelete(s_task);
    s_task = nullptr;
    s_newReading = false;

#ifdef FEATURE_GPIO_LEDS
    if (s_ledLinked)
        led_multi_set(s_ledPin, 0);
#endif

    LOG("[SENSOR] Stopped");
}

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────
void ultrasound_init()
{
    LOG("[SENSOR] Ultrasound ready"
        " | trig:" +
        String(s_trigPin) +
        " echo:" + String(s_echoPin) +
        " | Send sensor:start to begin");
}

bool ultrasound_is_running() { return s_task != nullptr; }
float ultrasound_get_distance_cm() { return s_distanceCm; }
uint8_t ultrasound_get_duty() { return s_lastDuty; }

// ─────────────────────────────────────────────────────────────────────────────
// handle() — called from loop() every iteration
// Reads shared state written by the FreeRTOS task and drives LED + MQTT.
// ─────────────────────────────────────────────────────────────────────────────
void ultrasound_handle()
{
    if (!s_task)
        return;

    uint32_t now = millis();

    // ── Periodic status publish (independent of new readings) ────────────
    if ((now - s_lastStatus) >= 5000)
    {
        s_lastStatus = now;
        float cm = s_distanceCm;
        String st = String(s_task ? "running" : "stopped") +
                    " dist=" + (cm < 0 ? String("oor") : String(cm, 1)) +
                    " duty=" + String(s_lastDuty) +
                    " linked=" + String(s_ledLinked ? 1 : 0);
        mqtt_publish(CFG_ULTRASOUND_TOPIC_STATUS, st);
    }

    if (!s_newReading)
        return;
    s_newReading = false;

    float cm = s_distanceCm;

    // ── LED brightness mapping ────────────────────────────────────────────
#ifdef FEATURE_GPIO_LEDS
    if (s_ledLinked && cm > 0)
    {
        uint8_t duty = distanceToDuty(cm);
        s_lastDuty = duty;
        led_multi_set(s_ledPin, duty);
    }
    else if (s_ledLinked && cm < 0)
    {
        // Out of range — extinguish LED to signal no-object
        s_lastDuty = 0;
        led_multi_set(s_ledPin, 0);
    }
#endif

    // ── Distance logging / publish ────────────────────────────────────────
    if (!s_loggingOn)
        return;

    bool passThreshold = (s_thresholdCm <= 0.0f) || (cm > 0 && cm <= s_thresholdCm);
    if (!passThreshold)
        return;

    if ((now - s_lastLog) >= s_logIntervalMs)
    {
        s_lastLog = now;

        String payload = (cm < 0) ? String("oor") : String(cm, 1);
        mqtt_publish(CFG_ULTRASOUND_TOPIC_DIST, payload);

        if (s_thresholdCm > 0.0f && cm > 0 && cm <= s_thresholdCm)
            LOGF("[SENSOR] ⚠ Threshold! %.1f cm (≤ %.0f cm)\n", cm, s_thresholdCm);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// MQTT command handler
// msg = full message, e.g. "sensor:start" | "sensor:ratio:5:80"
// ─────────────────────────────────────────────────────────────────────────────
static void sensorCmdHandler(const String &msg)
{
    // Token helper — 0-indexed field in colon-delimited msg.
    // tok(0) = "sensor", tok(1) = action, tok(2) = sub, tok(3) = value
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

    String action = tok(1);

    // ── sensor:start ──────────────────────────────────────────────────────
    if (action == "start")
    {
        startTask();
        return;
    }

    // ── sensor:stop ───────────────────────────────────────────────────────
    if (action == "stop")
    {
        stopTask();
        return;
    }

    // ── sensor:read — one-shot blocking read (only when task not running) ─
    if (action == "read")
    {
        if (s_task)
        {
            float cm = s_distanceCm;
            LOG("[SENSOR] Latest: " +
                (cm < 0 ? String("out of range") : String(cm, 1) + " cm") +
                " | duty:" + String(s_lastDuty));
            return;
        }
        // Manual one-shot on this thread (loop() must not be time-critical here)
        pinMode(s_trigPin, OUTPUT);
        pinMode(s_echoPin, INPUT);
        digitalWrite(s_trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(s_trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(s_trigPin, LOW);
        long d = pulseIn(s_echoPin, HIGH, CFG_ULTRASOUND_TIMEOUT_US);
        float cm = recordMeasurement(d);
        LOG("[SENSOR] One-shot: " +
            (cm < 0 ? String("timeout/out of range") : String(cm, 1) + " cm"));
        mqtt_publish(CFG_ULTRASOUND_TOPIC_DIST,
                     cm < 0 ? String("oor") : String(cm, 1));
        return;
    }

    // ── sensor:link:on / sensor:link:off ──────────────────────────────────
    if (action == "link")
    {
        String sub = tok(2);
#ifdef FEATURE_GPIO_LEDS
        if (sub == "on")
        {
            led_multi_set_mode(LedMultiMode::MANUAL); // must be MANUAL for set() to work
            s_ledLinked = true;
            LOG("[SENSOR] LED link ON → pin " + String(s_ledPin) +
                " | range " + String(s_minCm, 0) + "-" + String(s_maxCm, 0) + " cm" +
                " | invert:" + String(s_invertRatio ? "yes" : "no"));
        }
        else if (sub == "off")
        {
            s_ledLinked = false;
            led_multi_set(s_ledPin, 0);
            LOG("[SENSOR] LED link OFF");
        }
        else
        {
            LOG("[SENSOR] Usage: sensor:link:on | sensor:link:off");
        }
#else
        LOG("[SENSOR] LED link requires FEATURE_GPIO_LEDS in platformio.ini");
#endif
        return;
    }

    // ── sensor:pin:trig:<n> / sensor:pin:echo:<n> ─────────────────────────
    if (action == "pin")
    {
        String sub = tok(2);
        uint8_t pin = (uint8_t)tok(3).toInt();

        bool wasRunning = ultrasound_is_running();
        if (wasRunning)
            stopTask();

        if (sub == "trig")
        {
            s_trigPin = pin;
            LOG("[SENSOR] Trig pin → " + String(pin));
        }
        else if (sub == "echo")
        {
            s_echoPin = pin;
            LOG("[SENSOR] Echo pin → " + String(pin));
        }
        else
        {
            LOG("[SENSOR] Usage: sensor:pin:trig:<n> | sensor:pin:echo:<n>");
        }

        if (wasRunning)
            startTask(); // restart with new pins
        return;
    }

    // ── sensor:led:pin:<n> ────────────────────────────────────────────────
    if (action == "led")
    {
        String sub = tok(2);
        if (sub == "pin")
        {
            s_ledPin = (uint8_t)tok(3).toInt();
            LOG("[SENSOR] LED target pin → " + String(s_ledPin));
        }
        else
        {
            LOG("[SENSOR] Usage: sensor:led:pin:<n>");
        }
        return;
    }

    // ── sensor:ratio:<min>:<max>  OR  sensor:ratio:invert:<0|1> ──────────
    if (action == "ratio")
    {
        String sub = tok(2);
        if (sub == "invert")
        {
            s_invertRatio = (tok(3).toInt() != 0);
            LOG("[SENSOR] Ratio invert → " + String(s_invertRatio ? "on (near=dim)" : "off (near=bright)"));
            return;
        }
        // sensor:ratio:<min>:<max>
        float newMin = sub.toFloat();
        float newMax = tok(3).toFloat();
        if (newMin >= newMax)
        {
            LOG("[SENSOR] ratio: min must be less than max");
            return;
        }
        s_minCm = newMin;
        s_maxCm = newMax;
        LOG("[SENSOR] Ratio range → " + String(s_minCm, 0) + " cm (bright) to " +
            String(s_maxCm, 0) + " cm (off)");
        return;
    }

    // ── sensor:log:on | off | interval:<ms> ──────────────────────────────
    if (action == "log")
    {
        String sub = tok(2);
        if (sub == "on")
        {
            s_loggingOn = true;
            LOG("[SENSOR] Distance logging ON — interval " + String(s_logIntervalMs) + "ms");
        }
        else if (sub == "off")
        {
            s_loggingOn = false;
            LOG("[SENSOR] Distance logging OFF");
        }
        else if (sub == "interval")
        {
            uint32_t ms = (uint32_t)tok(3).toInt();
            if (ms < 50)
            {
                LOG("[SENSOR] Min log interval is 50ms");
                return;
            }
            s_logIntervalMs = ms;
            LOG("[SENSOR] Log interval → " + String(ms) + "ms");
        }
        else
        {
            LOG("[SENSOR] Usage: sensor:log:on | sensor:log:off | sensor:log:interval:<ms>");
        }
        return;
    }

    // ── sensor:threshold:<cm> — 0 disables ───────────────────────────────
    if (action == "threshold")
    {
        s_thresholdCm = tok(2).toFloat();
        if (s_thresholdCm <= 0.0f)
            LOG("[SENSOR] Threshold disabled");
        else
            LOG("[SENSOR] Threshold → " + String(s_thresholdCm, 0) + " cm");
        return;
    }

    // ── sensor:sample:<ms> ────────────────────────────────────────────────
    if (action == "sample")
    {
        uint32_t ms = (uint32_t)tok(2).toInt();
        if (ms < 50)
        {
            LOG("[SENSOR] Min sample interval is 50ms");
            return;
        }
        s_sampleMs = ms;
        // If task is running it picks up the new value on next vTaskDelay loop
        LOG("[SENSOR] Sample interval → " + String(ms) + "ms");
        return;
    }

    // ── sensor:status ─────────────────────────────────────────────────────
    if (action == "status")
    {
        float cm = s_distanceCm;
        LOGF("[SENSOR] running:%d  last:%.1fcm  duty:%u  "
             "trig:%u  echo:%u  ledPin:%u  "
             "range:%.0f-%.0fcm  invert:%d  linked:%d  "
             "log:%d  logInt:%ums  threshold:%.0fcm  sample:%ums\n",
             (int)ultrasound_is_running(), cm, s_lastDuty,
             s_trigPin, s_echoPin, s_ledPin,
             s_minCm, s_maxCm, (int)s_invertRatio, (int)s_ledLinked,
             (int)s_loggingOn, s_logIntervalMs, s_thresholdCm, s_sampleMs);
        return;
    }

    // ── sensor:health / sensor:resetstats ────────────────────────────────
    if (action == "health")
    {
        publishHealth();
        return;
    }

    if (action == "resetstats")
    {
        resetHealth();
        LOG("[SENSOR] Health counters reset");
        return;
    }

    LOG("[SENSOR] Commands: start | stop | read | link:on/off | "
        "pin:trig/echo:<n> | led:pin:<n> | ratio:<min>:<max> | "
        "ratio:invert:<0/1> | log:on/off | log:interval:<ms> | "
        "threshold:<cm> | sample:<ms> | status | health | resetstats");
}

void ultrasound_register_commands()
{
    cmd_register("sensor:", sensorCmdHandler);
}

#endif // FEATURE_ULTRASOUND
