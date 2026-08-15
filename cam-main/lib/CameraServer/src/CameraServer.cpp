#include "CameraServer.h"
#include "cam_config.h"
#include "logger.h"

#include <WiFi.h>
#include <esp_camera.h>
#include <esp_http_server.h>
#include <esp_timer.h>

CameraServer cameraServer;

// ── MJPEG multipart streaming (roadmap P2.1a) ────────────────────────────────
#define CAM_PART_BOUNDARY "123456789000000000000987654321"

static const char* CAM_STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" CAM_PART_BOUNDARY;
static const char* CAM_STREAM_BOUNDARY   = "\r\n--" CAM_PART_BOUNDARY "\r\n";
static const char* CAM_STREAM_PART_HEADER =
    "Content-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n";

static esp_err_t handleStreamRequest(httpd_req_t* req) {
    esp_err_t res = httpd_resp_set_type(req, CAM_STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;

    const int64_t framePeriodUs = (int64_t)(1000000.0f / (float)CAM_TARGET_FPS);
    uint32_t frameCount = 0;

    for (;;) {
        const int64_t frameStartUs = esp_timer_get_time();

        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            LOG_ERR("CAM: capture failed while streaming.");
            break;
        }

        uint8_t* jpgBuf = nullptr;
        size_t jpgBufLen = 0;
        bool ownsJpg = false;

        if (fb->format != PIXFORMAT_JPEG) {
            // Failsafe — the driver is configured for JPEG, so this is rare.
            ownsJpg = frame2jpg(fb, CAM_JPEG_QUALITY, &jpgBuf, &jpgBufLen);
            if (!ownsJpg) {
                LOG_ERR("CAM: JPEG conversion failed.");
                esp_camera_fb_return(fb);
                break;
            }
        } else {
            jpgBuf = fb->buf;
            jpgBufLen = fb->len;
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
        if (res != ESP_OK) break; // client gone / socket error → stop capturing

        // Soft FPS cap: yield CPU back to TaskNetwork (MQTT) on core 0.
        const int64_t elapsedUs = esp_timer_get_time() - frameStartUs;
        const int64_t remainingUs = framePeriodUs - elapsedUs;
        if (remainingUs > 0) {
            vTaskDelay(pdMS_TO_TICKS((TickType_t)(remainingUs / 1000)));
        }

        if ((++frameCount % 50) == 0) {
            const uint32_t fps = (uint32_t)((elapsedUs > 0) ? (1000000UL / (uint32_t)elapsedUs) : CAM_TARGET_FPS);
            LOG_NET("MJPG: %uKB/frame, %ufps (heap %u)",
                    (uint32_t)(jpgBufLen / 1024), fps, (uint32_t)ESP.getFreeHeap());
        }
    }

    httpd_resp_send_chunk(req, nullptr, 0); // terminate multipart cleanly
    return (res == ESP_OK) ? ESP_OK : ESP_FAIL;
}

// ── Camera init ──────────────────────────────────────────────────────────────
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
    }

    LOG_NET("CAM ready (PSRAM=%s, fb_count=%d)",
            psramFound() ? "yes" : "no", CAM_FB_COUNT);
    return true;
}

// ── HTTP server ──────────────────────────────────────────────────────────────
bool CameraServer::startServer(uint16_t port) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port     = port;
    cfg.max_open_sockets = 4;
    cfg.stack_size       = 8192;
    cfg.core_id          = 0; // ESP32: run the server task on core 0 (network core)

    httpd_handle_t server = nullptr;
    esp_err_t err = httpd_start(&server, &cfg);
    if (err != ESP_OK) {
        LOG_ERR("CAM: httpd_start failed: 0x%x", err);
        return false;
    }

    const httpd_uri_t streamUri = { "/stream", HTTP_GET, handleStreamRequest, nullptr };
    err = httpd_register_uri_handler(server, &streamUri);
    if (err != ESP_OK) {
        LOG_ERR("CAM: httpd_register_uri_handler failed: 0x%x", err);
        httpd_stop(server);
        return false;
    }

    m_server = server;
    m_running = true;
    LOG_NET("CAM: MJPEG ready on :%u/stream", port);
    return true;
}

bool CameraServer::begin(uint16_t port) {
    if (m_running) return true;
    const bool ok = initCamera() && startServer(port);
    if (!ok) LOG_ERR("CAM: camera subsystem failed to start.");
    return ok;
}