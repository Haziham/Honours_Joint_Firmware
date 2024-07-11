#include "joint.h"


Joint_t joint;

void joint_decodeCANPackets(Joint_t *joint, CAN_Message_t *canMessage)
{
    if (decodeJointCommandPacketStructure(canMessage, &joint->command) |
        decodeJointSettingsPacketStructure(canMessage, &joint->jointSettings) |
        decodeStatusAPacketStructure(canMessage, &joint->statusA) |
        decodeStatusBPacketStructure(canMessage, &joint->statusB) |
        decodeTelemetrySettingsPacketStructure(canMessage, &joint->telemetrySettings))
    {
        // Error decoding packet
    }
}
