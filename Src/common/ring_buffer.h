#ifndef RING_BUFFER_H
#define RING_BUFFER_H
/*Ring buffer for UART application*/
#include <stdint.h>
#include <stdbool.h>
struct ring_buffer
{
    uint8_t *buffer;
    uint8_t head, tail, size;
};
void ring_buffer_put(struct ring_buffer *rb, uint8_t data);
uint8_t ring_buffer_get(struct ring_buffer *rb);
uint8_t ring_buffer_peek(const struct ring_buffer *rb);
bool ring_buffer_full(const struct ring_buffer *rb);
bool ring_buffer_empty(const struct ring_buffer *rb);
#endif // RING_BUFFER_H