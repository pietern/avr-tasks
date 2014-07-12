#include <avr/io.h>
#include <stddef.h>

#include "uart.h"
#include "task.h"

void echo_task(void *unused) {
  char buf[16];
  FILE uart = FDEV_SETUP_STREAM(uart_putc, uart_getc, _FDEV_SETUP_RW);

  while (1) {
    if (fgets(buf, sizeof(buf), &uart) != NULL) {
      fputs(buf, &uart);
    }
  }
}

int main() {
  uart_init();

  task_initialize();

  task_create(echo_task, NULL);

  task_start();

  return 0; // Never reached
}
