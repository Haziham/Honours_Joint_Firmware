#include "joint.h"
#include "cmsis_os.h"

Joint_t joint;

void joint_decodeCANPackets(CAN_Message_t *canMessage)
{
    if (canMessage->len == 0)
    {
        // Its a packet request 
        send_requestedPacket(canMessage); 
    }
    else if (decodeJointCommandPacketStructure(canMessage, &joint.command) |
        decodeJointSettingsPacketStructure(canMessage, &joint.jointSettings) |
        decodeStatusAPacketStructure(canMessage, &joint.statusA) |
        decodeStatusBPacketStructure(canMessage, &joint.statusB) |
        decodeTelemetrySettingsPacketStructure(canMessage, &joint.telemetrySettings) |
        decodeCommandSettingsPacketStructure(canMessage, &joint.commandSettings)  |
        decodeEnablePacket(canMessage, &joint.statusA.enabled));
    else
    {
        joint.statusA.error = 1;
    }
}


void send_requestedPacket(CAN_Message_t *canMessage)
{
    // May need to optermise this for space
    // Can make use of the fact all settings is just a block of data
    switch (canMessage->id)
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
