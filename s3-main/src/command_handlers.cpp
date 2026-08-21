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
    });

    dispatcher.registerHandler(CMD_TYPE_SERVO, [&servoMgr, &motionCtrl](const JsonDocument& doc) {
        motionCtrl.setRawServoMode(true);
        uint8_t ch = doc["channel"] | 0;
        uint16_t rawPulse = doc["pulse_us"] | 1500;
        servoMgr.setServoPulseUs(ch, rawPulse);
        LOG_MOT("Direct Servo Write: Ch %d -> %d us", ch, rawPulse);
    });

    dispatcher.registerHandler(CMD_TYPE_SERVO_BATCH, [&servoMgr, &motionCtrl](const JsonDocument& doc) {
        motionCtrl.setRawServoMode(true);
        JsonArrayConst servos = doc["servos"].as<JsonArrayConst>();
        for (JsonObjectConst s : servos) {
            uint8_t ch = s["ch"] | 0;
            uint16_t rawPulse = s["pulse_us"] | 1500;
            servoMgr.setServoPulseUs(ch, rawPulse);
        }
        LOG_MOT("Executed servo_batch write (%d channels)", servos.size());
    });

    dispatcher.registerHandler(CMD_TYPE_MOTION, [&motionCtrl, &servoMgr](const JsonDocument& doc) {
        servoMgr.setOutputsEnabled(true);
        motionCtrl.setRawServoMode(false); 

        auto getF = [](const JsonDocument& d, const char* k1, const char* k2 = nullptr, const char* k3 = nullptr, const char* k4 = nullptr, float defVal = 0.0f) -> float {
            if (k1 && !d[k1].isNull()) return d[k1].as<float>();
            if (k2 && !d[k2].isNull()) return d[k2].as<float>();
            if (k3 && !d[k3].isNull()) return d[k3].as<float>();
            if (k4 && !d[k4].isNull()) return d[k4].as<float>();
            return defVal;
        };

        if (doc["gait"].is<const char*>()) {
            String gaitStr = doc["gait"].as<String>();
            gaitStr.toLowerCase();
            if (gaitStr == "ripple") motionCtrl.setGaitType(GaitType::RIPPLE);
            else if (gaitStr == "wave") motionCtrl.setGaitType(GaitType::WAVE);
            else motionCtrl.setGaitType(GaitType::TRIPOD);
        }

        VelocityCommand vCmd;
        vCmd.vx         = getF(doc, "vx", "Vx", nullptr, nullptr, 0.0f);
        vCmd.vy         = getF(doc, "vy", "Vy", nullptr, nullptr, 0.0f);
        vCmd.omega      = getF(doc, "omega", "w", "turn", nullptr, 0.0f);
        vCmd.stepHeight = getF(doc, "stepHeight", "step_height", "liftSwing", nullptr, 25.0f);
        vCmd.cycleTime  = getF(doc, "cycleTime", "cycle_time", "speed", nullptr, 1.0f);
        vCmd.hipStance  = getF(doc, "hipStance", "hip_stance", "hipSwing", nullptr, 0.0f);
        
        // INVERTED: Matches UI slider up = legs splay out / body lowers
        vCmd.legStance  = -getF(doc, "legStance", "leg_stance", "stance", nullptr, 0.0f); 
        
        motionCtrl.setVelocity(vCmd);

        float rawTx = getF(doc, "tx", "pos_x", "posX", "surge", 0.0f);
        float rawTy = getF(doc, "ty", "pos_y", "posY", "sway", 0.0f);
        float rawTz = getF(doc, "tz", "pos_z", "posZ", "heave", 0.0f);
        float rawRx = getF(doc, "rx", "pitch", "Pitch", nullptr, 0.0f);
        float rawRy = getF(doc, "ry", "roll",  "Roll",  nullptr, 0.0f);
        float rawRz = getF(doc, "rz", "yaw",   "Yaw",   nullptr, 0.0f);

        BodyPose pose;
        pose.posX  =  rawTx;   // Surge (Forward/Back)
        pose.posY  =  rawTy;   // Sway (Left/Right)
        pose.posZ  =  rawTz;   // Heave (tz UP moves Body UP)
        
        // ALL ROTATIONS INVERTED: Flips the standard math right-hand-rule to match your UI sliders
        pose.pitch = -rawRx;   // Pitch
        pose.roll  = -rawRy;   // Roll 
        pose.yaw   = -rawRz;   // Yaw
        
        motionCtrl.setBodyPose(pose);

        LOG_MOT("Live IK Updated: Stance[Leg=%.1f, Hip=%.1f] Pose[X=%.1f, Y=%.1f, Z=%.1f, R=%.1f, P=%.1f, Y=%.1f]",
                vCmd.legStance, vCmd.hipStance, pose.posX, pose.posY, pose.posZ, pose.roll, pose.pitch, pose.yaw);
    });

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

    dispatcher.registerHandler(CMD_TYPE_SYSTEM, [&mqttMgr, &servoMgr](const JsonDocument& doc) {
        if (doc["logging"].is<bool>()) g_logEnabled = doc["logging"].as<bool>();
        if (doc["command"].is<const char*>() && String(doc["command"].as<String>()) == "get_config") mqttMgr.sendConfig();
        if (doc["power"].is<bool>()) servoMgr.setOutputsEnabled(doc["power"].as<bool>());
    });

    dispatcher.registerHandler(CMD_TYPE_OTA, [&otaMgr](const JsonDocument& doc) {
        otaMgr.checkForUpdates(
            doc["primary"] | false, doc["fallback"] | false,
            doc["owner"] | "", doc["repo"] | "", doc["branch"] | "", 
            doc["project_path"] | "", doc["pat"] | ""
        );
    });
}