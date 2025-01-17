#include "can_queues.h"

void CAN_queue_init(CAN_Queue_t *queue, CAN_Message_t *data, uint8_t size)
{
    queue->data = data;
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
    queue->size = size;
}

uint8_t CAN_enqueue_message(CAN_Queue_t *queue, CAN_Message_t *message)
{

    if (CAN_queue_full(queue))
    {
        return ERROR; // Queue is full, return error.
    }

    queue->data[queue->tail] = *message;
    queue->tail = (queue->tail + 1) % queue->size;
    queue->count++;

    return SUCCESS; // Success
}

uint8_t CAN_dequeue_message(CAN_Queue_t *queue, CAN_Message_t *message)
{
    if (CAN_queue_empty(queue))
    {
        return ERROR; // Queue is empty, return error.
    }

    *message = queue->data[queue->head];
    queue->head = (queue->head + 1) % queue->size;
    queue->count--;

    return SUCCESS; // Success
}


uint8_t CAN_queue_empty(CAN_Queue_t *queue)
{
    return queue->count == 0;
}

uint8_t CAN_queue_full(CAN_Queue_t *queue)
{
    return queue->count == queue->size; 
}
