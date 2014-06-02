#include <avr/io.h>

#include "task.h"

volatile uint8_t delay_ms = 0;

void blink_task(void *data) {
  volatile uint8_t *ms = data;

  while (1) {
    task_sleep(*ms);
    PORTB ^= _BV(PB5);
  }
}

void delay_task(void *data) {
  volatile uint8_t *ms = data;

  while (1) {
    *ms = 50;
    task_sleep(1000);
    *ms = 200;
    task_sleep(1000);
  }
}

int main() {
  // PB5 (pin 13) is an output pin
  DDRB |= _BV(PB5);

  task_initialize();

  task_create(blink_task, (void *)&delay_ms);
  task_create(delay_task, (void *)&delay_ms);

  task_start();

  return 0; // Never reached
}
