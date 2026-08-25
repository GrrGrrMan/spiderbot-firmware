#include "CameraServer.h"
#include "cam_config.h"
#include "logger.h"

#include <WiFi.h>
#include <esp_camera.h>
#include <img_converters.h>
#include <esp_http_server.h>
#include <esp_timer.h>

CameraServer cameraServer;

#define CAM_PART_BOUNDARY "123456789000000000000987654321"

static const char* CAM_STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" CAM_PART_BOUNDARY;
static const char* CAM_STREAM_BOUNDARY = "\r\n--" CAM_PART_BOUNDARY "\r\n";
static const char* CAM_STREAM_PART_HEADER =
    "Content-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n";

static bool s_isStreamingActive = false;

static framesize_t parseFramesize(const char* name) {
    if (!name) return FRAMESIZE_VGA;
    if (strcasecmp(name, "96X96") == 0)   return FRAMESIZE_96X96;
    if (strcasecmp(name, "QQVGA") == 0)   return FRAMESIZE_QQVGA;
    if (strcasecmp(name, "QCIF") == 0)    return FRAMESIZE_QCIF;
    if (strcasecmp(name, "HQVGA") == 0)   return FRAMESIZE_HQVGA;
    if (strcasecmp(name, "240X240") == 0) return FRAMESIZE_240X240;
    if (strcasecmp(name, "QVGA") == 0)    return FRAMESIZE_QVGA;
    if (strcasecmp(name, "CIF") == 0)     return FRAMESIZE_CIF;
    if (strcasecmp(name, "HVGA") == 0)    return FRAMESIZE_HVGA;
    if (strcasecmp(name, "VGA") == 0)     return FRAMESIZE_VGA;
    if (strcasecmp(name, "SVGA") == 0)    return FRAMESIZE_SVGA;
    if (strcasecmp(name, "XGA") == 0)     return FRAMESIZE_XGA;
    if (strcasecmp(name, "HD") == 0)      return FRAMESIZE_HD;
    if (strcasecmp(name, "SXGA") == 0)    return FRAMESIZE_SXGA;
    if (strcasecmp(name, "UXGA") == 0)    return FRAMESIZE_UXGA;
    if (strcasecmp(name, "FHD") == 0)     return FRAMESIZE_FHD;   // OV3660 Native 1080p
    if (strcasecmp(name, "QXGA") == 0)    return FRAMESIZE_QXGA;  // OV3660 Max (2048x1536)
    return FRAMESIZE_VGA;
}

static esp_err_t handleStreamRequest(httpd_req_t* req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    if (s_isStreamingActive) {
        LOG_ERR("CAM: Connection rejected — another client is already streaming.");
        httpd_resp_set_status(req, "429 Too Many Requests");
        httpd_resp_send(req, "Stream in use", HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }
    s_isStreamingActive = true;

    esp_err_t res = httpd_resp_set_type(req, CAM_STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        s_isStreamingActive = false;
        return res;
    }

    uint32_t frameCount = 0;
    LOG_NET("CAM: Client connected to /stream");

    while (res == ESP_OK) {
        const int64_t frameStartUs = esp_timer_get_time();
        const uint8_t curFps = cameraServer.getTargetFps();
        const int64_t framePeriodUs = (int64_t)(1000000.0f / (float)max((uint8_t)1, curFps));

        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            LOG_ERR("CAM: Frame capture failed.");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        uint8_t* jpgBuf = fb->buf;
        size_t jpgBufLen = fb->len;
        bool ownsJpg = false;

        if (fb->format != PIXFORMAT_JPEG) {
            ownsJpg = frame2jpg(fb, CAM_JPEG_QUALITY, &jpgBuf, &jpgBufLen);
            if (!ownsJpg) {
                LOG_ERR("CAM: Non-JPEG conversion failed.");
                esp_camera_fb_return(fb);
                break;
            }
        }

        res = httpd_resp_send_chunk(req, CAM_STREAM_BOUNDARY, strlen(CAM_STREAM_BOUNDARY));
        if (res == ESP_OK) {
            char partHdr[64];
            int hlen = snprintf(partHdr, sizeof(partHdr), CAM_STREAM_PART_HEADER, jpgBufLen);
            res = (hlen > 0 && (size_t)hlen < sizeof(partHdr))
                      ? httpd_resp_send_chunk(req, partHdr, (size_t)hlen)
                      : ESP_FAIL;
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char*)jpgBuf, jpgBufLen);
        }

        if (ownsJpg) free(jpgBuf);
        esp_camera_fb_return(fb);

        if (res != ESP_OK) break;

        const int64_t elapsedUs = esp_timer_get_time() - frameStartUs;
        const int64_t remainingUs = framePeriodUs - elapsedUs;
        
        if (remainingUs > 1000) {
            vTaskDelay(pdMS_TO_TICKS((TickType_t)(remainingUs / 1000)));
        } else {
            taskYIELD();
        }

        if ((++frameCount % 50) == 0) {
            const uint32_t actualFps = (uint32_t)((elapsedUs > 0) ? (1000000UL / (uint32_t)elapsedUs) : curFps);
            LOG_NET("MJPG: %uKB/frame, ~%ufps (Free heap: %u)",
                    (uint32_t)(jpgBufLen / 1024), actualFps, (uint32_t)ESP.getFreeHeap());
        }
    }

    s_isStreamingActive = false;
    LOG_NET("CAM: Client disconnected from /stream");
    httpd_resp_send_chunk(req, nullptr, 0);
    return ESP_OK;
}

bool CameraServer::initFlashlight() {
    ledcSetup(CAM_LAMP_CHANNEL, 5000, 8); // 5 kHz PWM, 8-bit resolution (0-255)
    ledcAttachPin(CAM_PIN_LAMP, CAM_LAMP_CHANNEL);
    setFlashlight(0); // Start OFF
    return true;
}

void CameraServer::setFlashlight(uint8_t brightnessPercent) {
    m_lampBrightness = constrain(brightnessPercent, (uint8_t)0, (uint8_t)100);
    uint32_t duty = (uint32_t)((m_lampBrightness * 255) / 100);
    ledcWrite(CAM_LAMP_CHANNEL, duty);
    LOG_SYS("CAM: Flashlight -> %u%%", m_lampBrightness);
}

void CameraServer::setTargetFps(uint8_t fps) {
    m_targetFps = constrain(fps, (uint8_t)1, (uint8_t)30);
    LOG_NET("CAM: Stream Target FPS -> %u", m_targetFps);
}

bool CameraServer::applyCameraConfig(const JsonDocument& doc) {
    sensor_t* s = esp_camera_sensor_get();
    if (!s) {
        LOG_ERR("CAM: Cannot adjust sensor — camera driver not initialized.");
        return false;
    }

    // 0. Composite Presets
    if (doc["preset"].is<const char*>()) {
        const char* preset = doc["preset"].as<const char*>();
        if (strcasecmp(preset, "night_vision") == 0 || strcasecmp(preset, "night") == 0) {
            setFlashlight(80);
            s->set_exposure_ctrl(s, 1);
            s->set_gain_ctrl(s, 1);
            s->set_agc_gain(s, 20);
            s->set_special_effect(s, 2); // Grayscale
            setTargetFps(15);
            LOG_NET("CAM: Preset -> NIGHT_VISION");
        } else if (strcasecmp(preset, "inspection") == 0 || strcasecmp(preset, "macro") == 0) {
            setFlashlight(30);
            s->set_quality(s, 8); // High quality
            s->set_special_effect(s, 0); // Normal
            LOG_NET("CAM: Preset -> INSPECTION");
        } else if (strcasecmp(preset, "stealth") == 0) {
            setFlashlight(0);
            s->set_special_effect(s, 0);
            LOG_NET("CAM: Preset -> STEALTH");
        } else if (strcasecmp(preset, "low_power") == 0) {
            setFlashlight(0);
            setTargetFps(5);
            s->set_framesize(s, FRAMESIZE_QVGA);
            LOG_NET("CAM: Preset -> LOW_POWER");
        } else if (strcasecmp(preset, "default") == 0 || strcasecmp(preset, "reset") == 0) {
            setFlashlight(0);
            s->set_framesize(s, CAM_FRAME_SIZE);
            s->set_quality(s, CAM_JPEG_QUALITY);
            s->set_special_effect(s, 0);
            s->set_brightness(s, 0);
            s->set_contrast(s, 0);
            s->set_saturation(s, 0);
            setTargetFps(CAM_TARGET_FPS);
            LOG_NET("CAM: Preset -> DEFAULT (RESET)");
        }
    }

    // 1. Hardware Flashlight (0–100%)
    if (doc["flash"].is<int>())        setFlashlight(doc["flash"].as<int>());
    else if (doc["lamp"].is<int>())    setFlashlight(doc["lamp"].as<int>());
    else if (doc["led"].is<int>())     setFlashlight(doc["led"].as<int>());

    // 2. Stream Target Framerate (1–30 FPS)
    if (doc["fps"].is<int>())          setTargetFps(doc["fps"].as<int>());
    else if (doc["target_fps"].is<int>()) setTargetFps(doc["target_fps"].as<int>());

    // 3. Resolution / Framesize
    if (doc["framesize"].is<const char*>()) {
        framesize_t fs = parseFramesize(doc["framesize"].as<const char*>());
        s->set_framesize(s, fs);
        LOG_NET("CAM: Framesize -> %s (%d)", doc["framesize"].as<const char*>(), (int)fs);
    } else if (doc["framesize"].is<int>()) {
        s->set_framesize(s, (framesize_t)doc["framesize"].as<int>());
        LOG_NET("CAM: Framesize enum -> %d", doc["framesize"].as<int>());
    }

    // 4. JPEG Quality (0 - 63, lower is higher quality)
    if (doc["quality"].is<int>()) {
        int q = constrain(doc["quality"].as<int>(), 0, 63);
        s->set_quality(s, q);
        LOG_NET("CAM: Quality -> %d", q);
    }

    // 5. Brightness, Contrast, Saturation (-2 to 2)
    if (doc["brightness"].is<int>()) s->set_brightness(s, constrain(doc["brightness"].as<int>(), -2, 2));
    if (doc["contrast"].is<int>())   s->set_contrast(s, constrain(doc["contrast"].as<int>(), -2, 2));
    if (doc["saturation"].is<int>()) s->set_saturation(s, constrain(doc["saturation"].as<int>(), -2, 2));

    // 6. Exposure (AEC) & Gain (AGC)
    if (doc["exposure_ctrl"].is<bool>()) s->set_exposure_ctrl(s, doc["exposure_ctrl"].as<bool>() ? 1 : 0);
    if (doc["ae_level"].is<int>())      s->set_ae_level(s, constrain(doc["ae_level"].as<int>(), -2, 2));
    if (doc["aec_value"].is<int>())     s->set_aec_value(s, constrain(doc["aec_value"].as<int>(), 0, 1200));
    if (doc["aec2"].is<bool>())         s->set_aec2(s, doc["aec2"].as<bool>() ? 1 : 0);

    if (doc["gain_ctrl"].is<bool>())    s->set_gain_ctrl(s, doc["gain_ctrl"].as<bool>() ? 1 : 0);
    if (doc["agc_gain"].is<int>())      s->set_agc_gain(s, constrain(doc["agc_gain"].as<int>(), 0, 30));
    if (doc["gainceiling"].is<int>())   s->set_gainceiling(s, (gainceiling_t)constrain(doc["gainceiling"].as<int>(), 0, 6));

    // 7. White Balance (AWB)
    if (doc["whitebal"].is<bool>())     s->set_whitebal(s, doc["whitebal"].as<bool>() ? 1 : 0);
    if (doc["awb_gain"].is<bool>())     s->set_awb_gain(s, doc["awb_gain"].as<bool>() ? 1 : 0);
    if (doc["wb_mode"].is<int>())       s->set_wb_mode(s, constrain(doc["wb_mode"].as<int>(), 0, 4));

    // 8. Special Effects (0: None, 1: Negative, 2: Grayscale, 6: Sepia)
    if (doc["special_effect"].is<int>()) {
        s->set_special_effect(s, constrain(doc["special_effect"].as<int>(), 0, 6));
    }

    // 9. Orientation & Lens Correction
    if (doc["vflip"].is<bool>())        s->set_vflip(s, doc["vflip"].as<bool>() ? 1 : 0);
    if (doc["hmirror"].is<bool>())      s->set_hmirror(s, doc["hmirror"].as<bool>() ? 1 : 0);
    if (doc["lenc"].is<bool>())         s->set_lenc(s, doc["lenc"].as<bool>() ? 1 : 0);
    if (doc["bpc"].is<bool>())          s->set_bpc(s, doc["bpc"].as<bool>() ? 1 : 0);
    if (doc["wpc"].is<bool>())          s->set_wpc(s, doc["wpc"].as<bool>() ? 1 : 0);
    if (doc["raw_gma"].is<bool>())      s->set_raw_gma(s, doc["raw_gma"].as<bool>() ? 1 : 0);

    // 10. Sensor Windowing / Digital Zoom ([startX, startY, width, height])
    if (doc["crop"].is<JsonArrayConst>() && doc["crop"].as<JsonArrayConst>().size() == 4) {
        JsonArrayConst c = doc["crop"].as<JsonArrayConst>();
        int startX = c[0] | 0;
        int startY = c[1] | 0;
        int w = c[2] | 640;
        int h = c[3] | 480;
        s->set_res_raw(s, 0, 0, 0, 0, startX, startY, w, h, w, h, false, false);
        LOG_NET("CAM: Sensor window crop set to [%d, %d, %dx%d]", startX, startY, w, h);
    }

    return true;
}

bool CameraServer::initCamera() {
    camera_config_t cfg = {};
    cfg.ledc_channel   = (ledc_channel_t)CAM_LEDC_CHANNEL;
    cfg.ledc_timer     = (ledc_timer_t)CAM_LEDC_TIMER;
    cfg.pin_d0         = CAM_PIN_Y2;
    cfg.pin_d1         = CAM_PIN_Y3;
    cfg.pin_d2         = CAM_PIN_Y4;
    cfg.pin_d3         = CAM_PIN_Y5;
    cfg.pin_d4         = CAM_PIN_Y6;
    cfg.pin_d5         = CAM_PIN_Y7;
    cfg.pin_d6         = CAM_PIN_Y8;
    cfg.pin_d7         = CAM_PIN_Y9;
    cfg.pin_xclk       = CAM_PIN_XCLK;
    cfg.pin_pclk       = CAM_PIN_PCLK;
    cfg.pin_vsync      = CAM_PIN_VSYNC;
    cfg.pin_href       = CAM_PIN_HREF;
    cfg.pin_sccb_sda   = CAM_PIN_SIOD;
    cfg.pin_sccb_scl   = CAM_PIN_SIOC;
    cfg.pin_pwdn       = CAM_PIN_PWDN;
    cfg.pin_reset      = CAM_PIN_RESET;
    cfg.xclk_freq_hz   = CAM_XCLK_HZ;
    cfg.pixel_format   = PIXFORMAT_JPEG;
    cfg.frame_size     = CAM_FRAME_SIZE;
    cfg.jpeg_quality   = CAM_JPEG_QUALITY;
    cfg.fb_count       = CAM_FB_COUNT;
    cfg.grab_mode      = CAM_GRAB_MODE;
    cfg.fb_location    = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        LOG_ERR("CAM: esp_camera_init failed: 0x%x", err);
        return false;
    }

    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        s->set_framesize(s, CAM_FRAME_SIZE);
        s->set_quality(s, CAM_JPEG_QUALITY);

        // ── Set default inverted orientation ──
        s->set_vflip(s, CAM_DEFAULT_VFLIP);       // Vertical inversion
        s->set_hmirror(s, CAM_DEFAULT_HMIRROR);   // Horizontal inversion
    }

    initFlashlight();
    LOG_NET("CAM ready (OV3660, Inverted=%d/%d)", CAM_DEFAULT_VFLIP, CAM_DEFAULT_HMIRROR);
    return true;
}

bool CameraServer::startServer(uint16_t port) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port      = port;
    cfg.max_open_sockets = 2;
    cfg.stack_size       = 8192;
    cfg.core_id          = 1; 

    httpd_handle_t server = nullptr;
    esp_err_t err = httpd_start(&server, &cfg);
    if (err != ESP_OK) {
        LOG_ERR("CAM: httpd_start failed: 0x%x", err);
        return false;
    }

    const httpd_uri_t streamUri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = handleStreamRequest,
        .user_ctx  = nullptr
    };
    err = httpd_register_uri_handler(server, &streamUri);
    if (err != ESP_OK) {
        LOG_ERR("CAM: httpd_register_uri_handler failed: 0x%x", err);
        httpd_stop(server);
        return false;
    }

    m_server = server;
    m_running = true;
    LOG_NET("CAM: MJPEG server listening on Core 1 at :%u/stream", port);
    return true;
}

bool CameraServer::begin(uint16_t port) {
    if (m_running) return true;
    const bool ok = initCamera() && startServer(port);
    if (!ok) LOG_ERR("CAM: Camera subsystem failed to start.");
    return ok;
}