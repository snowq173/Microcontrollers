#include <stm32.h>
#include "messages_queue.h"


void clear_queue(messages_queue_t *queue) {
    queue->read_position = 0;
    queue->insert_position = 0;

    queue->used_space = 0;
}


uint8_t is_queue_empty(messages_queue_t *queue) {
    return queue->used_space == 0;
}


uint8_t is_queue_full(messages_queue_t *queue) {
    return queue->used_space == MESSAGES_QUEUE_BUFFER_SIZE;
}


void enqueue(messages_queue_t *queue, char *message) {
    queue->messages[queue->insert_position] = message;

    queue->insert_position = (queue->insert_position + 1) % MESSAGES_QUEUE_BUFFER_SIZE;
    queue->used_space++;
}


char *poll_queue(messages_queue_t *queue) {
    char *message = queue->messages[queue->read_position];

    queue->read_position = (queue->read_position + 1) % MESSAGES_QUEUE_BUFFER_SIZE;
    queue->used_space--;

    return message;
}
