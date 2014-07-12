#include <avr/io.h>
#include <avr/interrupt.h>
#include <stddef.h>
#include <util/twi.h>

#include "i2c.h"
#include "task.h"

struct i2c_op_s {
  uint8_t error;
  uint8_t address;
  uint8_t *buf;
  uint8_t pos;
  uint8_t len;
};

// Task waiting for I2C operation completion.
static task_t *i2c_task;

// Current I2C operation.
static struct i2c_op_s *i2c_op;

// By default, the control register is set to:
// - TWEA: Automatically send acknowledge bit in receive mode.
// - TWEN: Enable the I2C system.
// - TWIE: Enable interrupt requests when TWINT is set.
#define TWCR_DEFAULT (_BV(TWEA) | _BV(TWEN) | _BV(TWIE))

#define TWCR_ACK     (TWCR_DEFAULT |  _BV(TWINT))
#define TWCR_NOT_ACK (TWCR_ACK & ~_BV(TWEA))

#define TWCR_START (TWCR_DEFAULT | _BV(TWINT) | _BV(TWSTA))
#define TWCR_STOP  (TWCR_DEFAULT | _BV(TWINT) | _BV(TWSTO))

void i2c_init(void) {
  uint8_t sreg;

  sreg = SREG;
  cli();

  // From ATmega328p datasheet:
  //   SCL freq = (CPU Clock freq) / (16 + 2(TWBR) * (PrescalerValue))
  //
  // Which means:
  //   TWBR = ((CPU Clock freq) / (SCL freq) - 16) / (2 * (PrescalerValue))
  //
  // Disable the prescaler and set TWBR according to CPU freq and SCL freq.
  //
  TWSR &= ~(_BV(TWPS1) | _BV(TWPS0));
  TWBR = ((F_CPU / I2C_FREQ) - 16) / (2 * 1);

  // Active internal pull-up resistors for SCL and SDA.
  // Their ports are PC5 for SCL and PC4 for SDA on the ATmega328p.
  PORTC |= _BV(PC5) | _BV(PC4);

  // Enable the I2C system.
  TWCR = TWCR_DEFAULT;

  // Disable slave mode.
  TWAR = 0;

  SREG = sreg;
}

// Prepares I2C operation and suspends task to wait for completion.
static int8_t i2c__io(uint8_t address, uint8_t *buf, uint8_t len) {
  struct i2c_op_s op;
  uint8_t sreg;

  sreg = SREG;

  op.error = 0;
  op.address = address;
  op.buf = buf;
  op.pos = 0;
  op.len = len;

  cli();

  i2c_task = task_current();
  i2c_op = &op;

  // Transmit START to kickstart operation.
  TWCR = TWCR_START;

  task_suspend(NULL);

  SREG = sreg;

  if (op.error) {
    return -1;
  }

  return 0;
}

int8_t i2c_read(uint8_t address, uint8_t *buf, uint8_t len) {
  return i2c__io((address << 1) | TW_READ, buf, len);
}

int8_t i2c_write(uint8_t address, uint8_t *buf, uint8_t len) {
  return i2c__io((address << 1) | TW_WRITE, buf, len);
}

ISR(TWI_vect, ISR_BLOCK) {
  uint8_t status;

  status = TW_STATUS;

  switch (i2c_op->address & 0x1) {
  case TW_READ:
    // Master Receiver mode.
    switch (status) {

    // A START condition has been transmitted.
    case TW_START:
    // A repeated START condition has been transmitted.
    case TW_REP_START:
      i2c_op->pos = 0;
      TWDR = i2c_op->address;
      TWCR = TWCR_ACK;
      return;

    // Arbitration lost in SLA+R or NOT ACK bit.
    case TW_MR_ARB_LOST:
      // A START condition will be transmitted when the bus becomes free.
      TWCR = TWCR_START;
      return;

    // SLA+R has been transmitted; ACK has been received.
    case TW_MR_SLA_ACK:
      if (i2c_op->len == 1) {
        TWCR = TWCR_NOT_ACK;
      } else {
        TWCR = TWCR_ACK;
      }
      return;

    // SLA+R has been transmitted; NOT ACK has been received.
    case TW_MR_SLA_NACK:
      i2c_op->error = 1;
      goto done;

    // Data byte has been received; ACK has been returned.
    case TW_MR_DATA_ACK:
      i2c_op->buf[i2c_op->pos++] = TWDR;
      if (i2c_op->pos+1 == i2c_op->len) {
        TWCR = TWCR_NOT_ACK;
      } else {
        TWCR = TWCR_ACK;
      }
      return;

    // Data byte has been received; NOT ACK has been returned.
    case TW_MR_DATA_NACK:
      i2c_op->buf[i2c_op->pos++] = TWDR;
      goto done;
    }

    // Never reached, but be sure...
    return;

  case TW_WRITE:
    // Master Transmitter mode.
    switch (status) {

    // A START condition has been transmitted.
    case TW_START:
    // A repeated START condition has been transmitted.
    case TW_REP_START:
      i2c_op->pos = 0;
      TWDR = i2c_op->address;
      TWCR = TWCR_DEFAULT | _BV(TWINT);
      return;

    // Arbitration lost in SLA+W or data bytes.
    case TW_MT_ARB_LOST:
      // A START condition will be transmitted when the bus becomes free.
      TWCR = TWCR_START;
      return;

    // SLA+W has been transmitted; ACK has been received.
    case TW_MT_SLA_ACK:
      TWDR = i2c_op->buf[i2c_op->pos++];
      TWCR = TWCR_DEFAULT | _BV(TWINT);
      return;

    // SLA+W has been transmitted; NOT ACK has been received.
    case TW_MT_SLA_NACK:
      i2c_op->error = 1;
      goto done;

    // Data byte has been transmitted; ACK has been received.
    case TW_MT_DATA_ACK:
      if (i2c_op->pos < i2c_op->len) {
        TWDR = i2c_op->buf[i2c_op->pos++];
        TWCR = TWCR_DEFAULT | _BV(TWINT);
        return;
      }

      // No more bytes left to transmit...
      goto done;

    // Data byte has been transmitted; NOT ACK has been received.
    case TW_MT_DATA_NACK:
      if (i2c_op->pos < i2c_op->len) {
        // There were more bytes left to transmit!
        i2c_op->error = 1;
      }

      goto done;
    }

    // Never reached, but be sure...
    return;
  }

  // Never reached, but be sure...
  return;

done:
  TWCR = TWCR_STOP;
  task_wakeup(i2c_task);
  return;
}
