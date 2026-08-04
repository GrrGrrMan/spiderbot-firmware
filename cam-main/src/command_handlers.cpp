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
            servoMgr.setServoPulseUs(ch, rawPulse); // ✅ Clean single-line write per channel
        }
        LOG_MOT("Executed servo_batch write (%d channels)", servos.size());
    });

    // 3. Motion Engine Handler (Velocity Vectors & 6-DOF Body Poses)
    dispatcher.registerHandler(CMD_TYPE_MOTION, [&motionCtrl](const JsonDocument& doc) {
        motionCtrl.setRawServoMode(false); // RESUME the IK Engine
        // Check for Gait Mode Switch ("tripod", "ripple", "wave")
        if (doc["gait"].is<const char*>()) {
            String gaitStr = doc["gait"].as<String>();
            gaitStr.toLowerCase();
            if (gaitStr == "ripple") {
                motionCtrl.setGaitType(GaitType::RIPPLE);
                LOG_MOT("Gait switched to: RIPPLE");
            } else if (gaitStr == "wave") {
                motionCtrl.setGaitType(GaitType::WAVE);
                LOG_MOT("Gait switched to: WAVE");
            } else {
                motionCtrl.setGaitType(GaitType::TRIPOD);
                LOG_MOT("Gait switched to: TRIPOD");
            }
        }

        // Process Velocity Vector Command
        if (!doc["vx"].isNull() || !doc["vy"].isNull() || !doc["omega"].isNull() || !doc["leg_stance"].isNull()) {
            VelocityCommand vCmd;
            vCmd.vx         = doc["vx"]          | 0.0f;
            vCmd.vy         = doc["vy"]          | 0.0f;
            vCmd.omega      = doc["omega"]       | 0.0f;
            vCmd.stepHeight = doc["step_height"] | 25.0f;
            vCmd.cycleTime  = doc["cycle_time"]  | 1.0f;
            vCmd.legStance  = doc["leg_stance"]  | 0.0f;
            vCmd.hipStance  = doc["hip_stance"]  | 0.0f;
            motionCtrl.setVelocity(vCmd);
            LOG_MOT("Velocity Cmd: Vx=%.1f, Vy=%.1f, W=%.1f, LegStance=%.1f, HipStance=%.1f", 
                    vCmd.vx, vCmd.vy, vCmd.omega, vCmd.legStance, vCmd.hipStance);
        }

        // Process Body Pose Command
        if (!doc["roll"].isNull() || !doc["pitch"].isNull() || !doc["pos_z"].isNull()) {
            BodyPose pose;
            pose.posX  = doc["pos_x"] | 0.0f;
            pose.posY  = doc["pos_y"] | 0.0f;
            pose.posZ  = doc["pos_z"] | 0.0f;
            pose.roll  = doc["roll"]  | 0.0f;
            pose.pitch = doc["pitch"] | 0.0f;
            pose.yaw   = doc["yaw"]   | 0.0f;
            motionCtrl.setBodyPose(pose);
            LOG_MOT("Pose Cmd: Roll=%.1f, Pitch=%.1f, Yaw=%.1f", pose.roll, pose.pitch, pose.yaw);
        }
    });

    // 3.5 High-Level Pose Command Handler (Decoupled UI)
    dispatcher.registerHandler(CMD_TYPE_POSE, [&motionCtrl](const JsonDocument& doc) {
        motionCtrl.setRawServoMode(true); // Pause IK Engine for manual angle control
        JsonObjectConst poseObj = doc["pose"].as<JsonObjectConst>();
        
        // Maps firmware array indices (0-5) to the Web UI string names
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
        LOG_MOT("Applied direct angle pose from Web UI.");
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