#include "RingBuffer.h"

uint8_t Ring_Buffer_Push(RingBuffer_Info *ring_buffer, uint8_t data)
{
    uint8_t next;

    next = ring_buffer->pHead + 1; // next is where head will point to after this write.
    if (next >= ring_buffer->maxlen)
        next = 0;

    if (next == ring_buffer->pTail) // if the head + 1 == tail, circular buffer is full
        return -1;

    ring_buffer->buffer[ring_buffer->pHead] = data; // Load data and then move
    ring_buffer->pHead = next;                      // head to next data offset.
    return 0;                                       // return success to indicate successful push.
}

uint8_t Ring_Buffer_Pop(RingBuffer_Info *ring_buffer, uint8_t *data)
{
    uint8_t next;

    if (ring_buffer->pHead == ring_buffer->pTail) // if the head == tail, we don't have any data
        return -1;

    next = ring_buffer->pTail + 1; // next is where tail will point to after this read.
    if (next >= ring_buffer->maxlen)
        next = 0;

    *data = ring_buffer->buffer[ring_buffer->pTail]; // Read data and then move
    ring_buffer->pTail = next;                       // tail to next offset.
    return 0;                                        // return success to indicate successful push.
}

uint16_t Ring_Buffer_Available(RingBuffer_Info *ring_buffer)
{
    if (ring_buffer->pHead >= ring_buffer->pTail)
        return ring_buffer->pHead - ring_buffer->pTail;

    else
        return (ring_buffer->maxlen - ring_buffer->pTail) + ring_buffer->pHead;
}

void Ring_Buffer_Init(RingBuffer_Info *ring_buffer, uint8_t *buffer, uint16_t length)
{
    ring_buffer->pHead = 0;
    ring_buffer->pTail = 0;
    ring_buffer->buffer = buffer;
    ring_buffer->maxlen = length;
}