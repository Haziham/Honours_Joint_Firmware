#include "joint.h"


Joint_t joint;

void joint_decodeCANPackets(Joint_t *joint, CAN_Message_t *canMessage)
{
    if (decodeJointCommandPacketStructure(canMessage, &joint->command) |
        decodeJointSettingsPacketStructure(canMessage, &joint->settings) |
        decodeStatusAPacketStructure(canMessage, &joint->statusA) |
        decodeStatusBPacketStructure(canMessage, &joint->statusB))
    {
        // Error decoding packet
    }
}
