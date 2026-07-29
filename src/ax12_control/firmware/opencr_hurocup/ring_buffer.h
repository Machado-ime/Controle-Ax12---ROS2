#ifndef RING_BUFFER_H
#define RING_BUFFER_H

/* Porte 1:1 do original (firmware/src/ring_buffer.h do HuroCup) — nenhuma
 * mudança de lógica, é código puro sem dependência de hardware. */

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

typedef struct {
    uint8_t  buf[RING_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} ring_buf_t;

void     ring_buf_init(ring_buf_t *rb);
bool     ring_buf_push(ring_buf_t *rb, uint8_t byte);
bool     ring_buf_pop(ring_buf_t *rb, uint8_t *byte);
uint16_t ring_buf_available(const ring_buf_t *rb);
bool     ring_buf_empty(const ring_buf_t *rb);

#endif /* RING_BUFFER_H */
