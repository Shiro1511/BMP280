/**
 ******************************************************************************
 * @file           : RingBuffer.h
 * @brief          : Header file for ring buffer (circular buffer) implementation.
 *                   Provides APIs for initialization, push, pop, and checking
 *                   available data in the buffer.
 ******************************************************************************
 * @attention
 *
 * This module helps manage FIFO-style data storage using a circular buffer.
 * It is lightweight and suitable for embedded applications such as UART data buffering.
 *
 ******************************************************************************
 */
#ifndef _RINGBUFFER_H_
#define _RINGBUFFER_H_

#include <stdint.h>

/**
 * @brief  Ring buffer structure definition.
 */
typedef struct
{
    uint8_t *buffer; /*!< Pointer to the buffer array */
    uint16_t pHead;  /*!< Write pointer (head index) */
    uint16_t pTail;  /*!< Read pointer (tail index) */
    uint16_t maxlen; /*!< Maximum number of elements in the buffer */
} RingBuffer_Info;

/**
 * @brief  Initialize a ring buffer.
 * @param  ring_buffer: Pointer to RingBuffer_Info structure.
 * @param  buffer: Pointer to memory array to be used as the buffer.
 * @param  length: Size (capacity) of the buffer.
 * @retval None
 */
void Ring_Buffer_Init(RingBuffer_Info *ring_buffer, uint8_t *buffer, uint16_t length);

/**
 * @brief  Push (write) one byte into the ring buffer.
 * @param  ring_buffer: Pointer to RingBuffer_Info structure.
 * @param  data: Byte data to be written into the buffer.
 * @retval uint8_t:
 *         - 1 if successful (data written).
 *         - 0 if buffer is full (data not written).
 */
uint8_t Ring_Buffer_Push(RingBuffer_Info *ring_buffer, uint8_t data);

/**
 * @brief  Pop (read) one byte from the ring buffer.
 * @param  ring_buffer: Pointer to RingBuffer_Info structure.
 * @param  data: Pointer to variable where the read data will be stored.
 * @retval uint8_t:
 *         - 1 if successful (data read).
 *         - 0 if buffer is empty (no data available).
 */
uint8_t Ring_Buffer_Pop(RingBuffer_Info *ring_buffer, uint8_t *data);

/**
 * @brief  Get the number of available bytes currently stored in the buffer.
 * @param  ring_buffer: Pointer to RingBuffer_Info structure.
 * @retval uint16_t: Number of bytes available to read.
 */
uint16_t Ring_Buffer_Available(RingBuffer_Info *ring_buffer);

#endif /* _RINGBUFFER_H_ */