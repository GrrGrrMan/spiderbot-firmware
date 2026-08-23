#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <esp_camera.h>
#include "cam_config.h"

class CameraServer {
public:
    bool begin(uint16_t port = CAM_STREAM_PORT);
    bool isRunning() const { return m_running; }

    // Dynamic Sensor & Hardware Customization
    bool applyCameraConfig(const JsonDocument& doc);
    void setFlashlight(uint8_t brightnessPercent);
    void setTargetFps(uint8_t fps);

    uint8_t getTargetFps() const { return m_targetFps; }
    uint8_t getFlashlight() const { return m_lampBrightness; }
    sensor_t* getSensor() const { return esp_camera_sensor_get(); }

private:
    bool initCamera();
    bool initFlashlight();
    bool startServer(uint16_t port);

    bool m_running = false;
    void* m_server = nullptr;
    uint8_t m_targetFps = CAM_TARGET_FPS;
    uint8_t m_lampBrightness = 0;
};

extern CameraServer cameraServer;