// #pragma once
#ifndef __CAN_QUEUES_H
#define __CAN_QUEUES_H

#include <stdint.h>
#include "stm32f0xx_hal.h"


typedef struct
{
    uint32_t id;     // CAN ID
    uint8_t data[8]; // CAN data
    uint8_t len;     // CAN data length
} CAN_Message_t;


typedef struct
{
    CAN_Message_t *data;
    uint8_t head;
    uint8_t tail;
    uint8_t count; // Number of CAN_Message_t elements in the queue
    uint8_t size;  // Number of possible CAN_Message_t elements in the queue
} CAN_Queue_t;

void CAN_queue_init(CAN_Queue_t *queue, CAN_Message_t *data, uint8_t size);


uint8_t CAN_enqueue_message(CAN_Queue_t *queue, CAN_Message_t *message);
uint8_t CAN_dequeue_message(CAN_Queue_t *queue, CAN_Message_t *message);
uint8_t CAN_queue_empty(CAN_Queue_t *queue);
uint8_t CAN_queue_full(CAN_Queue_t *queue);

#endif // __CAN_QUEUES_H