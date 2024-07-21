#pragma once

#include "freckle_protocol.h"
#include "canQueue.h"
#include "main.h"
#include "can.h"
#include "spi.h"
#include "AT25DF011.h"

#define ENCODER_CPR 6 

typedef struct {
    uint8_t saveSettingsFlag;
} InternalSettings_t;

typedef struct {
    JointSettings_t jointSettings;
    TelemetrySettings_t telemetrySettings;
    StatusA_t statusA;
    StatusB_t statusB;
    StatusC_t statusC;
    JointCommand_t command;
    CommandSettings_t commandSettings;
    InternalSettings_t internalSettings;  
} Joint_t;

extern Joint_t joint;


void joint_decodeCANPackets(CAN_Message_t* canMessage); 
void send_requestedPacket(CAN_Message_t *canMessage);

void convert_countToAngle(int16_t *angle, int16_t count);
void set_motorPWM(int32_t pwm, uint8_t offset);


#define save_settings() flash_writeBytes((uint8_t*)&joint.jointSettings, sizeof(JointSettings_t))
#define load_settings() flash_readBytes((uint8_t*)&joint.jointSettings, sizeof(JointSettings_t))


void send_settings(void);