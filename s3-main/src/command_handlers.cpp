#include "command_handlers.h"
#include "cmd_schema.h"
#include "servo_config.h"
#include "logger.h"



void registerAllCommandHandlers(
    CommandDispatcher& dispatcher,
    ServoManager& servoMgr,
    OTAManager& otaMgr,
    MotionController& motionCtrl,
    MQTTManager& mqttMgr
) {
    dispatcher.registerHandler("heartbeat", [](const JsonDocument& doc) {
        // updates g_lastCmdTime in setup()
    });

    // 1. Single Servo Direct Write Handler
    dispatcher.registerHandler(CMD_TYPE_SERVO, [&servoMgr, &motionCtrl](const JsonDocument& doc) {
        motionCtrl.setRawServoMode(true);
        uint8_t ch = doc["channel"] | 0;
        uint16_t rawPulse = doc["pulse_us"] | 1500;
        servoMgr.setServoPulseUs(ch, rawPulse);
        LOG_MOT("Direct Servo Write: Ch %d -> %d us", ch, rawPulse);
    });

    // 2. Batch Servo Direct Write Handler
    dispatcher.registerHandler(CMD_TYPE_SERVO_BATCH, [&servoMgr, &motionCtrl](const JsonDocument& doc) {
        motionCtrl.setRawServoMode(true);
        
        JsonArrayConst servos = doc["servos"].as<JsonArrayConst>();
        for (JsonObjectConst s : servos) {
            uint8_t ch = s["ch"] | 0;
            uint16_t rawPulse = s["pulse_us"] | 1500;
            servoMgr.setServoPulseUs(ch, rawPulse); // 
        }
        LOG_MOT("Executed servo_batch write (%d channels)", servos.size());
    });

    // 3. Motion Engine Handler (Velocity Vectors, Stances & 6-DOF Body Poses)
    dispatcher.registerHandler(CMD_TYPE_MOTION, [&motionCtrl, &servoMgr](const JsonDocument& doc) {
        servoMgr.setOutputsEnabled(true);
        motionCtrl.setRawServoMode(false); // Ensure IK Engine is ACTIVE

        auto getF = [](const JsonDocument& d, const char* k1, const char* k2 = nullptr, const char* k3 = nullptr, float defVal = 0.0f) -> float {
            if (k1 && !d[k1].isNull()) return d[k1].as<float>();
            if (k2 && !d[k2].isNull()) return d[k2].as<float>();
            if (k3 && !d[k3].isNull()) return d[k3].as<float>();
            return defVal;
        };

        // Check for Gait Mode Switch
        if (doc["gait"].is<const char*>()) {
            String gaitStr = doc["gait"].as<String>();
            gaitStr.toLowerCase();
            if (gaitStr == "ripple") {
                motionCtrl.setGaitType(GaitType::RIPPLE);
            } else if (gaitStr == "wave") {
                motionCtrl.setGaitType(GaitType::WAVE);
            } else {
                motionCtrl.setGaitType(GaitType::TRIPOD);
            }
        }

        // Parse Velocity & Stance Parameters (REMOVED "rz" from omega to prevent collision with static yaw!)
        VelocityCommand vCmd;
        vCmd.vx         = getF(doc, "vx", "Vx");
        vCmd.vy         = getF(doc, "vy", "Vy");
        vCmd.omega      = getF(doc, "omega", "w", "turn", 0.0f);
        vCmd.stepHeight = getF(doc, "stepHeight", "step_height", "liftSwing", 25.0f);
        vCmd.cycleTime  = getF(doc, "cycleTime", "cycle_time", "speed", 1.0f);
        vCmd.legStance  = getF(doc, "legStance", "leg_stance", "stance", 0.0f);
        vCmd.hipStance  = getF(doc, "hipStance", "hip_stance", "hipSwing", 0.0f);
        motionCtrl.setVelocity(vCmd);

        // Parse 6-DOF Body Pose Shifts
        BodyPose pose;
        pose.posX  = getF(doc, "pos_x", "posX", "tx");
        pose.posY  = getF(doc, "pos_y", "posY", "ty");
        pose.posZ  = getF(doc, "pos_z", "posZ", "tz");
        pose.roll  = getF(doc, "roll",  "rx",   "Roll");
        pose.pitch = getF(doc, "pitch", "ry",   "Pitch");
        pose.yaw   = getF(doc, "yaw",   "rz",   "Yaw");
        motionCtrl.setBodyPose(pose);

        LOG_MOT("Live IK Updated: Stance[Leg=%.1f, Hip=%.1f] Pose[X=%.1f, Y=%.1f, Z=%.1f, R=%.1f, P=%.1f, Y=%.1f]",
                vCmd.legStance, vCmd.hipStance, pose.posX, pose.posY, pose.posZ, pose.roll, pose.pitch, pose.yaw);
    });

    // 3.5 High-Level Pose Command Handler (Manual Servo Tab Only)
    dispatcher.registerHandler(CMD_TYPE_POSE, [&motionCtrl, &servoMgr](const JsonDocument& doc) {
        if (doc["pose"].is<JsonObjectConst>()) {
            servoMgr.setOutputsEnabled(true);
            motionCtrl.setRawServoMode(true);
            JsonObjectConst poseObj = doc["pose"].as<JsonObjectConst>();
            
            const char* legNames[6] = {
                "rightFront", "rightMiddle", "rightBack", 
                "leftBack", "leftMiddle", "leftFront"
            };
            
            for (uint8_t i = 0; i < 6; i++) {
                if (poseObj[legNames[i]].is<JsonObjectConst>()) {
                    float alpha = poseObj[legNames[i]]["alpha"] | 0.0f;
                    float beta  = poseObj[legNames[i]]["beta"]  | 0.0f;
                    float gamma = poseObj[legNames[i]]["gamma"] | 0.0f;
                    motionCtrl.setRawLegAngles(i, alpha, beta, gamma);
                }
            }
        }
    });
    // 4. System Logging Handler
    dispatcher.registerHandler(CMD_TYPE_SYSTEM, [&mqttMgr, &servoMgr](const JsonDocument& doc) {
        if (doc["logging"].is<bool>()) {
            g_logEnabled = doc["logging"].as<bool>();
            LOG_SYS("Logging state: %d", g_logEnabled);
        }
        if (doc["command"].is<const char*>()) {
            String cmd = doc["command"].as<String>();
            if (cmd == "get_config") {
                mqttMgr.sendConfig();
                LOG_SYS("Configuration handshake published on request.");
            }
        }
        if (doc["power"].is<bool>()) {
            bool powerState = doc["power"].as<bool>();
            servoMgr.setOutputsEnabled(powerState);
        }
    });

    // 5. OTA Update Handler
    dispatcher.registerHandler(CMD_TYPE_OTA, [&otaMgr](const JsonDocument& doc) {
        bool forceFallback = doc["fallback"]     | false;
        bool forcePrimary  = doc["primary"]      | false;

        String customOwner = doc["owner"]        | "";
        String customRepo  = doc["repo"]         | "";
        String customBranch= doc["branch"]       | "";
        String customPath  = doc["project_path"] | "";
        String customPat   = doc["pat"]          | "";

        LOG_SYS("Remote OTA command received via MQTT!");
        otaMgr.checkForUpdates(
            forcePrimary, forceFallback,
            customOwner, customRepo, customBranch, customPath, customPat
        );
    });
}