#include <avr/io.h>
#include <stddef.h>

#include "task.h"

volatile uint8_t delay_ms = 0;

void blink_task(void *unused) {
  while (1) {
    task_sleep(delay_ms);
    PORTB ^= _BV(PB5);
  }
}

void delay_task(void *unused) {
  while (1) {
    delay_ms = 50;
    task_sleep(1000);
    delay_ms = 200;
    task_sleep(1000);
  }
}

int main() {
  // PB5 (pin 13) is an output pin
  DDRB |= _BV(PB5);

  task_initialize();

  task_create(blink_task, NULL);
  task_create(delay_task, NULL);

  task_start();

  return 0; // Never reached
}
