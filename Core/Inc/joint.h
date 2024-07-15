#pragma once

#include "freckle_protocol.h"
#include "canQueue.h"
#include "main.h"
#include "can.h"

typedef struct {
    JointSettings_t jointSettings;
    TelemetrySettings_t telemetrySettings;
    StatusA_t statusA;
    StatusB_t statusB;
    JointCommand_t command;
    CommandSettings_t commandSettings;
} Joint_t;

extern Joint_t joint;


void joint_decodeCANPackets(CAN_Message_t* canMessage); 
void send_requestedPacket(CAN_Message_t *canMessage);