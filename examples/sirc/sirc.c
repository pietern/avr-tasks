#include <avr/io.h>
#include <stddef.h>
#include <util/delay.h>

#include <drivers/sirc.h>
#include <mutex.h>
#include <task.h>
#include <uart.h>

void read_loop(void *unused) {
  FILE uart = FDEV_SETUP_STREAM(uart_putc, uart_getc, _FDEV_SETUP_RW);
  uint16_t code;
  uint8_t device;
  uint8_t command;

  while (1) {
    code = sirc_read();
    device = (code >> 7) & 0x1f;
    command = code & 0x7f;
    fprintf(&uart, "Device: %d, command: %d\r\n", device, command);
  }
}

int main() {
  uart_init();

  sirc_init();

  task_init();

  task_create(read_loop, NULL);

  task_start();

  return 0; // Never reached
}
