#pragma once

#include "CommandDispatcher.h"
#include "OTAManager.h"
#include "MQTTManager.h"
#include "CameraServer.h"

void registerAllCommandHandlers(
    CommandDispatcher& dispatcher,
    OTAManager& otaMgr,
    MQTTManager& mqttMgr,
    CameraServer& camServer
);