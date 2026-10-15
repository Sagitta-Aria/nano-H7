#ifndef SERIAL_H
#define SERIAL_H
#include <stdint.h>
#define SERIAL_PORT_COUNT 2U
#define SERIAL_COM 0U
#define SERIAL_WIRELESS 1U
void serial_init(void);
/* Independent RX streams: USART1=0, UART7=1. Returns -1 when empty. */
int serial_read(unsigned port);
/* Broadcast one complete response to both UARTs, from the comm task only. */
int serial_write(const char *data);
uint32_t serial_rx_bytes(void);
uint32_t serial_rx_errors(void);
uint32_t serial_tx_errors(void);
#endif
