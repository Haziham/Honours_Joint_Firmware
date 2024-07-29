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
} InternalFlags_t;

typedef struct {
    JointSettings_t joint;
    TelemetrySettings_t telemetry;
    CommandSettings_t command;
} Settings_t;


typedef struct {
    Settings_t settings;
    StatusA_t statusA;
    StatusB_t statusB;
    StatusC_t statusC;
    JointCommand_t command;
    InternalFlags_t internalFlags;  
} Joint_t;

extern Joint_t joint;


void joint_decodeCANPackets(CAN_Message_t* canMessage); 
void send_requestedPacket(CAN_Message_t *canMessage);

void convert_countToAngle(int16_t *angle, int16_t count);
void set_motorPWM(int32_t pwm, uint8_t offset);

#define save_settings() flash_writeBytes((uint8_t*)&joint, sizeof(Settings_t))
#define load_settings() flash_readBytes((uint8_t*)&joint, sizeof(Settings_t))


void send_settings(void);