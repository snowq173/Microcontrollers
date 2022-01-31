#ifndef MESSAGES_QUEUE_H
#define MESSAGES_QUEUE_H


#define MESSAGES_QUEUE_BUFFER_SIZE                512


typedef struct {
    char *messages[MESSAGES_QUEUE_BUFFER_SIZE];
    uint32_t read_position;
    uint32_t insert_position;
    uint32_t used_space;
} messages_queue_t;


void clear_queue(messages_queue_t *);


uint8_t is_queue_empty(messages_queue_t *);


uint8_t is_queue_full(messages_queue_t *);


void enqueue(messages_queue_t *, char *);


char *poll_queue(messages_queue_t *);


#endif /* MESSAGES_QUEUE_H */
