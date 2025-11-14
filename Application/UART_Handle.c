#include "UART_Handle.h"

uint8_t uart_buffer_data[UART_BUFFER_SIZE];
RingBuffer_Info uart_ring_buffer;

void UART_Receive_Rx(uint8_t *data)
{
    Ring_Buffer_Push(&uart_ring_buffer, *data);
}

uint16_t UART_Handle_Available(void)
{
    return Ring_Buffer_Available(&uart_ring_buffer);
}

uint8_t UART_Handle_Read(void)
{
    uint8_t data;

    if (Ring_Buffer_Pop(&uart_ring_buffer, &data) == 0)
        return data;
    else
        return -1; /* Return -1 if no data available */
}

void UART_Handle_Init(void)
{
    Ring_Buffer_Init(&uart_ring_buffer, uart_buffer_data, UART_BUFFER_SIZE);
}
