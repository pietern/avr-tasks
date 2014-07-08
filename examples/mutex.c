#include <avr/io.h>
#include <stddef.h>

#include "mutex.h"
#include "task.h"

volatile uint8_t delay_ms = 0;

void blink_task(void *unused) {
  while (1) {
    task_sleep(delay_ms);
    PORTB ^= _BV(PB5);
  }
}

// The mutex sequences multiple tasks trying to set a blink interval.
mutex_t m;

void delay_task(void *data) {
  while (1) {
    mutex_lock(&m);
    delay_ms = (uint16_t) data;
    task_sleep(1000);
    mutex_unlock(&m);
  }
}

int main() {
  // PB5 (pin 13) is an output pin
  DDRB |= _BV(PB5);

  mutex_init(&m);

  task_initialize();

  task_create(blink_task, NULL);

  task_create(delay_task, (void *)20);
  task_create(delay_task, (void *)50);
  task_create(delay_task, (void *)100);

  task_start();

  return 0; // Never reached
}
