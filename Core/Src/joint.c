#include "joint.h"
#include "tim.h"
#include "cmsis_os.h"

Joint_t joint;

void joint_decodeCANPackets(CAN_Message_t *canMessage)
{
    if (canMessage->len == 0)
    {
        // Its a packet request 
        send_requestedPacket(canMessage); 
    }
    else if (   decodeJointCommandPacketStructure(canMessage, &joint.command) |
                decodeEnablePacket(canMessage, &joint.statusA.enabled)) 
    {

    }
    else if (   decodeJointSettingsPacketStructure(canMessage, &joint.jointSettings) |
                decodeTelemetrySettingsPacketStructure(canMessage, &joint.telemetrySettings) |
                decodeCommandSettingsPacketStructure(canMessage, &joint.commandSettings)) 
    {
        // Signal that settings should be saved
        joint.internalSettings.saveSettingsFlag = 1;
    }
    else
    {
        joint.statusA.error = 1;
    }
}

void send_settings(void)
{
    CAN_Message_t canMessage;
    encodeJointSettingsPacketStructure(&canMessage, &joint.jointSettings);
    CAN_SendMessage(&canMessage);

    encodeTelemetrySettingsPacketStructure(&canMessage, &joint.telemetrySettings);
    CAN_SendMessage(&canMessage);

    encodeCommandSettingsPacketStructure(&canMessage, &joint.commandSettings);
    CAN_SendMessage(&canMessage);
}

void send_requestedPacket(CAN_Message_t *canMessage)
{
    // May need to optimise this for space
    // Can make use of the fact all settings is just a block of data
    switch (getFrecklePacketID(canMessage))
    {
    case PKT_JOINT_COMMAND:
        encodeJointCommandPacketStructure(canMessage, &joint.command);
        break;
    case PKT_JOINT_SETTINGS:
        encodeJointSettingsPacketStructure(canMessage, &joint.jointSettings);
        break;
    case PKT_STATUSA:
        encodeStatusAPacketStructure(canMessage, &joint.statusA);
        break;
    case PKT_STATUSB:
        encodeStatusBPacketStructure(canMessage, &joint.statusB);
        break;
    case PKT_TELEMETRY_SETTINGS:
        encodeTelemetrySettingsPacketStructure(canMessage, &joint.telemetrySettings);
        break;
    case PKT_COMMAND_SETTINGS:
        encodeCommandSettingsPacketStructure(canMessage, &joint.commandSettings);
        break;
    default:
        break;
        joint.statusA.error = 1;
        return;
    }

    CAN_SendMessage(canMessage);
}

// Angle is in 0.1 degrees
void convert_countToAngle(int16_t *angle, int16_t count)
{
    *angle = (count * 360 * 10) / (ENCODER_CPR * joint.jointSettings.gearRatio);
}



void set_motorPWM(int32_t pwm, uint8_t offset)
{
    uint8_t direction = pwm > 0 ? DIR_FORWARD : DIR_BACKWARD;
    pwm = pwm < 0 ? -pwm : pwm;
    
    pwm += offset ? 5000 : 0;
    pwm = pwm > 65535 ? 65535 : pwm;
    pwm = 65535 - pwm;

    if (direction == DIR_FORWARD)
    {
        __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 65535);
        __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, pwm);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pwm);
        __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 65535);
    }

}

