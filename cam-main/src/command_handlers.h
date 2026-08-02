#pragma once

#include "CommandDispatcher.h"
#include "ServoManager.h"
#include "OTAManager.h"
#include "MotionController.h"

void registerAllCommandHandlers(
    CommandDispatcher& dispatcher,
    ServoManager& servoMgr,
    OTAManager& otaMgr,
    MotionController& motionCtrl
);