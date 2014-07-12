#include <avr/io.h>
#include <stddef.h>

#include "i2c.h"
#include "uart.h"
#include "task.h"

#define HMC5883L_ADDRESS (0x1e)

void hmc5883l_measure(FILE *uart) {
  uint8_t b3[1] = { 0x3 };
  uint8_t data[6];
  int16_t axis[3];

  // Read measurement
  i2c_write(HMC5883L_ADDRESS, b3, 1);
  i2c_read(HMC5883L_ADDRESS, data, 6);

  // Post-process measurement
  axis[0 /* X */] = (data[0] << 8) | data[1];
  axis[2 /* Z */] = (data[2] << 8) | data[3];
  axis[1 /* Y */] = (data[4] << 8) | data[5];

  fprintf(uart, "X: %5d, Y: %5d, Z: %5d\n", axis[0], axis[1], axis[2]);
}

void hmc5883l_task(void *unused) {
  uint8_t b0[2] = { 0x0, _BV(4) };
  uint8_t b1[2] = { 0x1, _BV(6) };
  uint8_t b2[2] = { 0x2, 0 };
  FILE uart = FDEV_SETUP_STREAM(uart_putc, uart_getc, _FDEV_SETUP_RW);

  // Initialize for continuous measurement at 15Hz
  i2c_write(HMC5883L_ADDRESS, b0, 2);
  i2c_write(HMC5883L_ADDRESS, b1, 2);
  i2c_write(HMC5883L_ADDRESS, b2, 2);

  while (1) {
    uint8_t t1, t2, ms;

    t1 = task_ms();
    hmc5883l_measure(&uart);
    t2 = task_ms();

    // This subtraction underflows if t1 > t2, but yields the right result.
    // Use measured time to space calls to "hmc5883l_measure" 100ms apart.
    ms = t2 - t1;
    task_sleep(100 - ms);
  }
}

int main() {
  i2c_init();
  uart_init();

  task_initialize();

  task_create(hmc5883l_task, NULL);

  task_start();

  return 0; // Never reached
}
