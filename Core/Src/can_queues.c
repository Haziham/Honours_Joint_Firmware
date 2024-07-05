#include "can_queues.h"

void CAN_queue_init(CAN_Queue_t *queue, CAN_Message_t *data, uint8_t size, osMutexId mutex)
{
    queue->data = data;
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
    queue->size = size;
    queue->mutex = mutex;
}

uint8_t CAN_enqueue_message(CAN_Queue_t *queue, CAN_Message_t *message)
{
    xSemaphoreTake(queue->mutex, portMAX_DELAY); // Take the mutex

    if (CAN_queue_full(queue))
    {
        xSemaphoreGive(queue->mutex); // Give the mutex
        return ERROR; // Queue is full, return error.
    }

    queue->data[queue->tail] = *message;
    queue->tail = (queue->tail + 1) % queue->size;
    queue->count++;

    xSemaphoreGive(queue->mutex); // Give the mutex
    return SUCCESS; // Success
}

uint8_t CAN_dequeue_message(CAN_Queue_t *queue, CAN_Message_t *message)
{
    xSemaphoreTake(queue->mutex, portMAX_DELAY); // Take the mutex
    if (CAN_queue_empty(queue))
    {
        xSemaphoreGive(queue->mutex); // Give the mutex
        return 1; // Queue is empty, return error.
    }

    *message = queue->data[queue->head];
    queue->head = (queue->head + 1) % queue->size;
    queue->count--;

    xSemaphoreGive(queue->mutex); // Give the mutex
    return 0; // Success
}


uint8_t CAN_queue_empty(CAN_Queue_t *queue)
{
    return queue->count == 0;
}

uint8_t CAN_queue_full(CAN_Queue_t *queue)
{
    return queue->count == queue->size; 
}
