#include <string.h>

#include "task.h"
#include "uart.h"
#include "readline.h"

void echo_task(void *unused) {
  uint16_t i;
  char prompt[16];
  char buf[16];
  int8_t len;

  for (i = 1;; i++) {
    sprintf(prompt, "%u> ", i);

    len = readline(prompt, buf, sizeof(buf));
    if (len > 0) {
      uart_putc('"', NULL);
      uart_write(buf, len);
      uart_putc('"', NULL);
      uart_write("\r\n", 2);
    }
  }
}

int main() {
  // 115200 for 16MHz clock
  uart_init(16, 1);

  task_init();

  task_create(echo_task, NULL);

  task_start();

  return 0; // Never reached
}
