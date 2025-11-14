/**
 ******************************************************************************
 * @file           : UART_Handle.h
 * @brief          : Header file for UART data handling using ring buffer.
 *                   Provides APIs for initializing UART, receiving data,
 *                   checking available bytes, and reading received data.
 ******************************************************************************
 * @attention
 *
 * This module uses a ring buffer to handle UART RX data asynchronously.
 * It can be easily integrated with HAL UART interrupt callbacks.
 *
 * Typical usage:
 *   1. Call UART_Handle_Init() once at startup.
 *   2. In HAL_UART_RxCpltCallback(), call UART_Receive_Rx().
 *   3. In main loop, use UART_Handle_Available() and UART_Handle_Read()
 *      to process incoming data.
 *
 ******************************************************************************
 */

#ifndef _UART_HANDLE_H_
#define _UART_HANDLE_H_

#include "stm32f1xx_hal.h"
#include "RingBuffer.h"

#define UART_BUFFER_SIZE 64 /*!< Size of internal UART RX buffer */

/**
 * @brief  Handle received UART byte and push it into the RX ring buffer.
 * @param  data: Pointer to the received byte (usually from HAL_UART_RxCpltCallback()).
 * @retval None
 */
void UART_Receive_Rx(uint8_t *data);

/**
 * @brief  Get number of bytes currently available in the UART RX buffer.
 * @retval uint16_t: Number of bytes available to read.
 */
uint16_t UART_Handle_Available(void);

/**
 * @brief  Read one byte from the UART RX buffer.
 * @retval uint8_t: The byte read from the buffer.
 * @note   If buffer is empty, function may return 0 or last value (depending on implementation).
 */
uint8_t UART_Handle_Read(void);

/**
 * @brief  Initialize UART RX ring buffer and start first reception.
 * @retval None
 */
void UART_Handle_Init(void);

#endif /* _UART_HANDLE_H_ */
