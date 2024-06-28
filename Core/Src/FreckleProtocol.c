#include "FreckleProtocol.h"
#include "can_queues.h"

uint8_t *getFrecklePacketData(void *pkt)
{
    CAN_Message_t *canMessage = (CAN_Message_t *)pkt;
    return canMessage->data;
}

const uint8_t *getFrecklePacketDataConst(const void *pkt)
{
    CAN_Message_t *canMessage = (CAN_Message_t *)pkt;
    return canMessage->data;
}

void finishFrecklePacket(void *pkt, int size, uint32_t packetID)
{
    CAN_Message_t *canMessage = (CAN_Message_t *)pkt;
    canMessage->len = size;
    canMessage->id = packetID;
}

int getFrecklePacketSize(const void *pkt)
{
    CAN_Message_t *canMessage = (CAN_Message_t *)pkt;
    return canMessage->len;
}

uint32_t getFrecklePacketID(const void *pkt)
{
    CAN_Message_t *canMessage = (CAN_Message_t *)pkt;
    return canMessage->id;
}
