/* Porte 1:1 do original — lógica idêntica, zero acesso a hardware. */
#include "ring_buffer.h"

void ring_buf_init(ring_buf_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
}

bool ring_buf_push(ring_buf_t *rb, uint8_t byte)
{
    uint16_t next_head = (rb->head + 1U) & (RING_BUF_SIZE - 1U);
    if (next_head == rb->tail) {
        return false;   /* full — drop byte */
    }
    rb->buf[rb->head] = byte;
    rb->head = next_head;
    return true;
}

bool ring_buf_pop(ring_buf_t *rb, uint8_t *byte)
{
    if (rb->tail == rb->head) {
        return false;   /* empty */
    }
    *byte = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1U) & (RING_BUF_SIZE - 1U);
    return true;
}

uint16_t ring_buf_available(const ring_buf_t *rb)
{
    return (rb->head - rb->tail) & (RING_BUF_SIZE - 1U);
}

bool ring_buf_empty(const ring_buf_t *rb)
{
    return rb->head == rb->tail;
}
