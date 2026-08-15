#pragma once
#include <Arduino.h>

// ── V2 Hexapod — CameraServer (P2 MJPEG stream) ─────────────────────────────
// Wraps the esp32-camera driver + esp_http_server to serve a browser-native
// MJPEG stream at http://<ip>:81/stream (multipart/x-mixed-replace).
// The whole subsystem is started from a FreeRTOS task pinned to core 0 so the
// 100 Hz control loop (core 1) + MQTT watchdog are never starved by capture.
// Frames are only captured while a browser is connected → idle CAM costs ~0.
class CameraServer {
public:
    // Initialize camera + start HTTP server. Returns false on init failure.
    bool begin(uint16_t port = CAM_STREAM_PORT);
    bool isRunning() const { return m_running; }

private:
    bool initCamera();
    bool startServer(uint16_t port);

    bool m_running = false;
    void* m_server = nullptr; // httpd_handle_t (kept opaque to keep header light)
};

extern CameraServer cameraServer;