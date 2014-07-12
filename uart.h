#ifndef _UART_H
#define _UART_H

#include <stdio.h>

#define UART_BAUD 9600

void uart_init(void);

int uart_putc(char c, FILE *unused);

int uart_getc(FILE *unused);

#endif
