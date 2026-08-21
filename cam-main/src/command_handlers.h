#pragma once

#include "CommandDispatcher.h"
#include "OTAManager.h"
#include "MQTTManager.h"

void registerAllCommandHandlers(
    CommandDispatcher& dispatcher,
    OTAManager& otaMgr,
    MQTTManager& mqttMgr
);