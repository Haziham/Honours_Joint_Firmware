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

