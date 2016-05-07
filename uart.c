#include <avr/io.h>
#include <avr/interrupt.h>
#include <stddef.h>

#include "uart.h"
#include "task.h"

#define _B(x, on) ((on) * _BV(x))

static task_t *tx_task = NULL;
static const uint8_t *tx_buf;
static uint8_t tx_count;

static task_t *rx_task = NULL;
static uint8_t *rx_buf;
static uint8_t rx_count;

#ifdef UART_COUNT_TX_BYTES
uint16_t uart_tx_bytes = 0;
#endif

#ifdef UART_COUNT_RX_BYTES
uint16_t uart_rx_bytes = 0;
#endif

#ifdef UART_COUNT_RX_ERRORS
uint8_t uart_rx_fe = 0;
uint8_t uart_rx_dor = 0;
uint8_t uart_rx_pe = 0;
uint8_t uart_rx_bdor = 0;
#endif

#define PRIV_BUF_SIZE (1<<4)
#define PRIV_BUF_SIZE_MASK ((PRIV_BUF_SIZE)-1)
#define PRIV_BUF_VAL(var) ((var) & PRIV_BUF_SIZE_MASK)

// Toggle the MSB every time an index wraps.
// This tells ppos and cpos apart if they are equal (empty vs full).
#define PRIV_BUF_INCR(var) do {       \
  (var)++;                            \
  if (PRIV_BUF_VAL(var) == 0) {       \
    (var) = (~((var) & 0x80) & 0x80); \
  }                                   \
} while (0);

// Private receive buffer.
static uint8_t rx_priv_buf[PRIV_BUF_SIZE];
static uint8_t rx_priv_ppos; // Producer
static uint8_t rx_priv_cpos; // Consumer

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
void uart_init(uint16_t ubrr, uint8_t x2) {
  uint8_t sreg;

  sreg = SREG;
  cli();

  UBRR0 = ubrr;

  UCSR0A = 0;
  UCSR0B = 0;
  UCSR0C = 0;

  // Double the transmission speed
  if (x2) {
    UCSR0A |= _B(U2X0, 1);
  }

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

  // Enable USART RX Complete Interrupt
  UCSR0B |= _B(RXCIE0, 1);

  SREG = sreg;
}

// Transmit interrupt handler.
ISR(USART_UDRE_vect) {
#ifdef UART_COUNT_TX_BYTES
  // The TX counter should be incremented from the TX complete interrupt
  // handler, but it is overkill to have a handler just for this.
  uart_tx_bytes++;
#endif
  UDR0 = tx_buf[0];
  if (--tx_count > 0) {
    tx_buf++;
  } else {
    // Disable USART Data Register Empty Interrupt
    UCSR0B &= ~_B(UDRIE0, 1);
    task_wakeup(tx_task);
  }
}

// Write data to UART.
int uart_write(const void *buf, size_t count) {
  uint8_t sreg;

  sreg = SREG;
  cli();

  tx_task = task_current();
  tx_buf = buf;
  tx_count = count;

  // Enable USART Data Register Empty Interrupt
  // It is disabled by the interrupt handler when done.
  UCSR0B |= _B(UDRIE0, 1);

  // Task is woken up by the interrupt handler when done.
  task_suspend(NULL);

  SREG = sreg;

  return count;
}

// Receive interrupt handler.
ISR(USART_RX_vect) {
  // Check for receive errors.
  if (UCSR0A & (_BV(FE0) | _BV(DOR0) | _BV(UPE0))) {
#ifdef UART_COUNT_RX_ERRORS
    if (UCSR0A & _BV(DOR0)) {
      uart_rx_fe++;
    }
    if (UCSR0A & _BV(DOR0)) {
      uart_rx_dor++;
    }
    if (UCSR0A & _BV(UPE0)) {
      uart_rx_pe++;
    }
#endif

    // Read data register to acknowledge interrupt.
    uint8_t tmp __attribute__((unused)) = UDR0;
    return;
  }

#ifdef UART_COUNT_RX_BYTES
  // No errors; byte was successfully read.
  uart_rx_bytes++;
#endif

  // Read into private buffer if external read buffer is not set.
  if (rx_buf == NULL) {
    rx_priv_buf[PRIV_BUF_VAL(rx_priv_ppos)] = UDR0;
    PRIV_BUF_INCR(rx_priv_ppos);

    // Advance consumer position if producer caught up.
    if (PRIV_BUF_VAL(rx_priv_ppos) == PRIV_BUF_VAL(rx_priv_cpos)) {
      PRIV_BUF_INCR(rx_priv_cpos);
#ifdef UART_COUNT_RX_ERRORS
      uart_rx_bdor++;
#endif
    }

    return;
  }

  rx_buf[0] = UDR0;
  if (--rx_count > 0) {
    rx_buf++;
  } else {
    rx_buf = NULL;
    task_wakeup(rx_task);
  }
}

// Read data from UART.
int uart_read(void *buf, size_t count) {
  uint8_t *bbuf = buf;
  uint8_t sreg;
  int n = 0;

  sreg = SREG;
  cli();

  // Read from private receive buffer if non-empty.
  while (count && rx_priv_cpos != rx_priv_ppos) {
    bbuf[0] = rx_priv_buf[PRIV_BUF_VAL(rx_priv_cpos)];
    PRIV_BUF_INCR(rx_priv_cpos);

    bbuf++;
    count--;
    n++;
  }

  // Wait for interrupt handler to populate remaining bytes.
  if (count) {
    rx_task = task_current();
    rx_buf = bbuf;
    rx_count = count;
    n += count;

    // Task is woken up by the interrupt handler when done.
    task_suspend(NULL);
  }

  SREG = sreg;
  return n;
}

// Read data from read buffer.
int uart_read_nonblock(void *buf, size_t count) {
    uint8_t *bbuf = buf;
    uint8_t sreg;
    int n = 0;

    sreg = SREG;
    cli();

    // Read from private receive buffer if non-empty.
    while (count && rx_priv_cpos != rx_priv_ppos) {
        bbuf[0] = rx_priv_buf[PRIV_BUF_VAL(rx_priv_cpos)];
        PRIV_BUF_INCR(rx_priv_cpos);

        bbuf++;
        count--;
        n++;
    }

    SREG = sreg;
    return n;
}

// Write character to UART.
int uart_putc(char c, FILE *unused) {
  uart_write(&c, 1);
  return 0;
}

// Read character from UART.
int uart_getc(FILE *unused) {
  char c;
  uart_read(&c, 1);
  return c;
}
