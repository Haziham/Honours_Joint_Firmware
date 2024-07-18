#pragma once

#include "freckle_protocol.h"
#include "canQueue.h"
#include "main.h"
#include "can.h"

#define ENCODER_CPR 6 

typedef struct {
    uint16_t previousPosition;
    uint16_t currentPosition;
    int16_t deltaPosition;
} JointInternalState_t; 


typedef struct {
    JointSettings_t jointSettings;
    TelemetrySettings_t telemetrySettings;
    StatusA_t statusA;
    StatusB_t statusB;
    StatusC_t statusC;
    JointCommand_t command;
    CommandSettings_t commandSettings;
    JointInternalState_t internalState;
} Joint_t;

extern Joint_t joint;


void joint_decodeCANPackets(CAN_Message_t* canMessage); 
void send_requestedPacket(CAN_Message_t *canMessage);

void convert_countToAngle(int16_t *angle, int16_t count);
void set_motorPWM(int32_t pwm, uint8_t offset);