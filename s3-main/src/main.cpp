# V2 Hexapod S3 Main — Minimal placeholder main.cpp
#
# This is the scaffolding entry point for the ESP32-S3 controller firmware.
# See firmware/s3-main/README.md for the build sequence and docs/future-roadmap/PLAN.md for architecture context.
#
# Phase P6 will flesh this into a full 100 Hz control loop with:
#   - LegIK, GaitGenerator, MotionController (ported from cam-main/lib/)
#   - ServoManager driving 2x PCA9685 via I2C (shared bus, 0x40/0x41)
#   - MQTTManager for hexapod/{id}/cmd subscription + telemetry publish
#   - MAX98357 I2S audio output (BCLK=GPIO40, LRC=GPIO39, DIN=GPIO38)
#   - HC-SR04 ultrasonic polling (TRIG=GPIO4, ECHO=GPIO5)

#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("S3 Hexapod Controller — booting...");
    Serial.println("Phase P6: servo control + sensors + audio not yet implemented");
    Serial.println("This is the scaffolding skeleton.");
}

void loop() {
    Serial.print(".");
    delay(1000);
}