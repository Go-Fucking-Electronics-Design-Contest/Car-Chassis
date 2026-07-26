#ifndef TJC_UART_H
#define TJC_UART_H

#include <stdint.h>

#define TJC_UART_RX_BUFFER_SIZE    (256u)
#define TJC_UART_TX_BUFFER_SIZE    (256u)

void TJC_UART_Init(void);

uint8_t TJC_UART_SendCommand(const char *cmd);

uint8_t TJC_UART_ReadByte(uint8_t *data);

extern volatile uint32_t tjc_uart_rx_byte_count;
extern volatile uint32_t tjc_uart_rx_overflow_count;
extern volatile uint32_t tjc_uart_tx_byte_count;
extern volatile uint32_t tjc_uart_tx_overflow_count;
extern volatile uint32_t tjc_uart_error_count;

#endif
