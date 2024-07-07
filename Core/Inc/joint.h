#pragma once

#include "freckle_protocol.h"
#include "can_queues.h"

typedef struct {
    JointSettings_t settings;
    StatusA_t statusA;
    StatusB_t statusB;
    JointCommand_t command;
} Joint_t;

extern Joint_t joint;


void joint_decodeCANPackets(Joint_t* joint, CAN_Message_t* canMessage); 
