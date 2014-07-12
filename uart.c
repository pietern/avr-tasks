#include <avr/io.h>
#include <avr/interrupt.h>
#include <stddef.h>

#include "uart.h"
#include "task.h"

#define _B(x, on) ((on) * _BV(x))

// Static variables for a transmitter.
static task_t *tx_task;
static char tx_buf;

// Static variables for a receiver.
static task_t *rx_task;
static char rx_buf;

// Register descriptions as documented in the ATmega328p datasheet.
//
// Bits in UCSR0A, USART Control and Status Register 0A
// RXC0:  USART Receive Complete
// TXC0:  USART Transmit Complete
// UDRE0: USART Data Register Empty
// FE0:   Frame Error
// DOR0:  Data Overrun
// UPE0:  USART Parity Error
// U2X0:  Double the USART Transmission Speed
// MPCM0: Multi-processor Communication Mode
//
// Bits in UCSR0B, USART Control and Status Register 0B
// RXCIE0: RX Complete Interrupt Enable
// TXCIE0: TX Complete Interrupt Enable
// UDRIE0: USART Data Register Empty Interrupt Enable
// RXEN0:  Receiver Enable
// TXEN0:  Transmitter Enable
// UCSZ02: Character Size bit 2
// RXB80:  Receive Data Bit 8
// TXB80:  Transmit Data Bit 8
//
// Bits in UCSR0C, USART Control and Status Register 0C
// UMSEL01: USART Mode Select bit 1
// UMSEL00: USART Mode Select bit 0
// UPM01:   Parity Mode bit 1
// UPM00:   Parity Mode bit 0
// USBS0:   Stop Bit Select
// UCSZ01:  Character Size bit 1
// UCSZ00:  Character Size bit 0
// UCPOL0:  Clock Polarity
//

// Some of the assignments here evaluate to 0 making them a no-op.
// They are included as documentation.
void uart_init(void) {
  uint8_t sreg;

  sreg = SREG;
  cli();

  UBRR0 = (F_CPU / (16UL * UART_BAUD)) - 1;

  UCSR0A = 0;
  UCSR0B = 0;
  UCSR0C = 0;

  // Asynchronous USART
  UCSR0C |= _B(UMSEL01, 0) | _B(UMSEL00, 0);

  // 8 bit character size
  UCSR0B |= _B(UCSZ02, 0);
  UCSR0C |= _B(UCSZ01, 1) | _B(UCSZ00, 1);

  // No parity bit
  UCSR0C |= _B(UPM01, 0) | _B(UPM00, 0);

  // 1 stop bit
  UCSR0C |= _B(USBS0, 0);

  // Enable RX/TX
  UCSR0B |= _B(RXEN0, 1) | _B(TXEN0, 1);

  SREG = sreg;
}

// Transmit interrupt handler.
ISR(USART_UDRE_vect) {
  UDR0 = tx_buf;

  // Disable USART Data Register Empty Interrupt
  UCSR0B &= ~_B(UDRIE0, 1);
  task_wakeup(tx_task);
}

// Transmit character over USART.
// Highly inefficient because of excessive context switching, but good enough
// for low throughput transmission at low baud rates.
int uart_putc(char c, FILE *unused) {
  uint8_t sreg;

  sreg = SREG;
  cli();

  tx_task = task_current();
  tx_buf = c;

  // Enable USART Data Register Empty Interrupt
  // It is disabled by the interrupt handler when done.
  UCSR0B |= _B(UDRIE0, 1);

  // Task is woken up by the interrupt handler when done.
  task_suspend(NULL);

  SREG = sreg;

  return 0;
}

// Receive interrupt handler.
ISR(USART_RX_vect) {
  rx_buf = UDR0;

  // Disable USART RX Complete Interrupt
  UCSR0B &= ~_B(RXCIE0, 1);
  task_wakeup(rx_task);
}

// Receive character over USART.
// Highly inefficient because of excessive context switching, but good enough
// for low throughput receival at low baud rates.
int uart_getc(FILE *unused) {
  uint8_t sreg;
  char c;

  sreg = SREG;
  cli();

  rx_task = task_current();

  // Enable USART RX Complete Interrupt
  // It is disabled by the interrupt handler when done.
  UCSR0B |= _B(RXCIE0, 1);

  // Task is woken up by the interrupt handler when done.
  task_suspend(NULL);

  c = rx_buf;

  SREG = sreg;

  return c;
}
