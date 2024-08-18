#pragma once

#include "freckle_protocol.h"
#include "canQueue.h"
#include "main.h"
#include "can.h"
#include "spi.h"
#include "AT25DF011.h"

#define ENCODER_CPR 12
#define STOP_POINT_OFFSET 50 // The 0.1s of degrees offset form stop points


#define CALIBRATE_START 0
#define FIND_MIN 1
#define FIND_MAX 2
#define CALIBRATE_END 3

#define CALIBRATION_PWM 10000

typedef struct {
    uint8_t saveSettingsFlag;
    uint8_t calibrateStep;
} InternalFlags_t;

typedef struct {
    uint32_t position;
} InternalSettings_t;

typedef struct {
    JointSettings_t joint;
    TelemetrySettings_t telemetry;
    CommandSettings_t command;
    CalibrationSettings_t calibration;
    ControlSettings_t control;
    InternalSettings_t internal;
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

void joint_init(void);

void joint_decodeCANPackets(CAN_Message_t* canMessage); 
void send_requestedPacket(CAN_Message_t *canMessage);

void convert_countToAngle(int16_t *angle, int16_t count);
void set_motorPWM(int32_t pwm, uint8_t offset);

#define save_settings() flash_writeBytes((uint8_t*)&joint, sizeof(Settings_t))
#define load_settings() flash_readBytes((uint8_t*)&joint, sizeof(Settings_t))


void send_settings(void);


void joint_calibrate(int32_t* pwm, uint32_t position, int16_t velocity);
uint8_t joint_isPastStopPoint(int16_t angle);