#ifndef _UART_H
#define _UART_H

#include <stdint.h>
#include <stdio.h>

#ifdef UART_COUNTERS
#define UART_COUNT_TX_BYTES
#define UART_COUNT_RX_BYTES
#define UART_COUNT_RX_ERRORS
#endif

#ifdef UART_COUNT_TX_BYTES
extern uint16_t uart_tx_bytes;
#endif

#ifdef UART_COUNT_RX_BYTES
extern uint16_t uart_rx_bytes;
#endif

#ifdef UART_COUNT_RX_ERRORS
// Frame Error
extern uint8_t uart_rx_fe;
// Data OverRun
extern uint8_t uart_rx_dor;
// Parity Error
extern uint8_t uart_rx_pe;
// Buffer Data OverRun (of software buffer)
extern uint8_t uart_rx_bdor;
#endif

void uart_init(uint16_t ubrr, uint8_t x2);

int uart_write(const void *buf, size_t count);

int uart_read(void *buf, size_t count);

int uart_read_nonblock(void *buf, size_t count);

int uart_putc(char c, FILE *unused);

int uart_getc(FILE *unused);

#endif
