#include <avr/io.h>
#include <avr/interrupt.h>
#include <stddef.h>
#include <util/twi.h>

#include "i2c.h"
#include "task.h"

struct i2c_op_s {
  uint8_t error;
  uint8_t address;
  struct i2c_iovec_s *iov;
  uint8_t iovcnt;
};

// Task waiting for I2C operation completion.
static task_t *i2c_task;

// Current I2C operation.
static volatile struct i2c_op_s i2c_op;

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

void i2c_open(void) {
  // No-op for now.
}

void i2c_close(void) {
  TWCR = TWCR_STOP;
}

// Prepares I2C operation and suspends task to wait for completion.
static int8_t i2c__io(uint8_t address, struct i2c_iovec_s *iov, uint8_t iovcnt) {
  uint8_t sreg;

  sreg = SREG;

  i2c_op.error = 0;
  i2c_op.address = address;
  i2c_op.iov = iov;
  i2c_op.iovcnt = iovcnt;

  cli();

  i2c_task = task_current();

  // Transmit START to kickstart operation.
  TWCR = TWCR_START;

  task_suspend(NULL);

  SREG = sreg;

  if (i2c_op.error) {
    i2c_close();
    return -1;
  }

  return 0;
}

int8_t i2c_readv(uint8_t address, struct i2c_iovec_s *iov, uint8_t iovcnt) {
  int8_t rv = i2c__io((address << 1) | TW_READ, iov, iovcnt);
  return rv;
}

int8_t i2c_writev(uint8_t address, struct i2c_iovec_s *iov, uint8_t iovcnt) {
  int8_t rv = i2c__io((address << 1) | TW_WRITE, iov, iovcnt);
  return rv;
}

int8_t i2c_read(uint8_t address, uint8_t *buf, uint8_t len) {
  struct i2c_iovec_s iov = { buf, len };
  int8_t rv = i2c_readv(address, &iov, 1);
  return rv;
}

int8_t i2c_write(uint8_t address, uint8_t *buf, uint8_t len) {
  struct i2c_iovec_s iov = { buf, len };
  int8_t rv = i2c_writev(address, &iov, 1);
  return rv;
}

int8_t i2c_read_from(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t len) {
  int8_t rv;

  rv = i2c_write(address, &reg, sizeof(reg));
  if (rv < 0) {
    return rv;
  }

  rv = i2c_read(address, buf, len);
  return rv;
}

int8_t i2c_write_to(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t len) {
  struct i2c_iovec_s iov[2] = { { &reg, 1 }, { buf, len } };
  int8_t rv = i2c_writev(address, (struct i2c_iovec_s *) &iov, 2);
  return rv;
}

ISR(TWI_vect, ISR_BLOCK) {
  uint8_t status;

  status = TW_STATUS;

  if ((i2c_op.address & 0x1) == TW_READ) {
    // TW_READ: Master Receiver mode.
    switch (status) {

    // A START condition has been transmitted.
    case TW_START:
    // A repeated START condition has been transmitted.
    case TW_REP_START:
      TWDR = i2c_op.address;
      TWCR = TWCR_ACK;
      break;

    // Arbitration lost in SLA+R or NOT ACK bit.
    case TW_MR_ARB_LOST:
      // A START condition will be transmitted when the bus becomes free.
      TWCR = TWCR_START;
      break;

    // SLA+R has been transmitted; ACK has been received.
    case TW_MR_SLA_ACK:
      // Return NACK after next byte if it is the last one.
      if (i2c_op.iov[0].len == 1 && i2c_op.iovcnt == 1) {
        TWCR = TWCR_NOT_ACK;
      } else {
        TWCR = TWCR_ACK;
      }
      break;

    // SLA+R has been transmitted; NOT ACK has been received.
    case TW_MR_SLA_NACK:
      i2c_op.error = 1;
      goto done;

    // Data byte has been received; ACK has been returned.
    case TW_MR_DATA_ACK:
      i2c_op.iov[0].base[0] = TWDR;
      i2c_op.iov[0].base++;
      if (--i2c_op.iov[0].len == 0) {
        i2c_op.iov++;
        i2c_op.iovcnt--;
        // iovcnt is > 0, or we would have run TW_MR_DATA_NACK.
      }

      // Return NACK after next byte if it is the last one.
      if (i2c_op.iov[0].len == 1 && i2c_op.iovcnt == 1) {
        TWCR = TWCR_NOT_ACK;
      } else {
        TWCR = TWCR_ACK;
      }
      break;

    // Data byte has been received; NOT ACK has been returned.
    case TW_MR_DATA_NACK:
      i2c_op.iov[0].base[0] = TWDR;
      goto done;

    default:
      // Don't know what to do now...
      i2c_op.error = 1;
      goto done;
    }
  } else {
    // TW_WRITE: Master Transmitter mode.
    switch (status) {

    // A START condition has been transmitted.
    case TW_START:
    // A repeated START condition has been transmitted.
    case TW_REP_START:
      TWDR = i2c_op.address;
      TWCR = TWCR_DEFAULT | _BV(TWINT);
      break;

    // Arbitration lost in SLA+W or data bytes.
    case TW_MT_ARB_LOST:
      // A START condition will be transmitted when the bus becomes free.
      TWCR = TWCR_START;
      break;

    // SLA+W has been transmitted; ACK has been received.
    case TW_MT_SLA_ACK:
      TWDR = i2c_op.iov[0].base[0];
      TWCR = TWCR_DEFAULT | _BV(TWINT);
      i2c_op.iov[0].base++;
      i2c_op.iov[0].len--;
      break;

    // SLA+W has been transmitted; NOT ACK has been received.
    case TW_MT_SLA_NACK:
      i2c_op.error = 1;
      goto done;

    // Data byte has been transmitted; ACK has been received.
    case TW_MT_DATA_ACK:
      if (i2c_op.iov[0].len == 0) {
        i2c_op.iov++;
        if (--i2c_op.iovcnt == 0) {
          // No more bytes left to transmit...
          goto done;
        }
      }

      TWDR = i2c_op.iov[0].base[0];
      TWCR = TWCR_DEFAULT | _BV(TWINT);
      i2c_op.iov[0].base++;
      i2c_op.iov[0].len--;
      break;

    // Data byte has been transmitted; NOT ACK has been received.
    case TW_MT_DATA_NACK:
      if (i2c_op.iov[0].len == 0) {
        i2c_op.iov++;
        if (--i2c_op.iovcnt == 0) {
          // No more bytes left to transmit...
          goto done;
        }
      }

      // There were more bytes left to transmit!
      i2c_op.error = 1;
      goto done;

    default:
      // Don't know what to do now...
      i2c_op.error = 1;
      goto done;
    }
  }

  return;

done:
  // From the ATmega328p datasheet (section 21.9.2):
  //
  //   The TWINT Flag must be cleared by software by writing a logic one to it.
  //   Note that this flag is not automatically cleared by hardware when
  //   executing the interrupt routine. Also note that clearing this flag
  //   starts the operation of the TWI, so all accesses to the TWI Address
  //   Register (TWAR), TWI Status Register (TWSR), and TWI Data Register
  //   (TWDR) must be complete before clearing this flag.
  //
  // It is up to the code that uses I2C functions whether it wants to bundle
  // multiple operations in one atomic set or not (this is done using repeated
  // start). This means that TWINT cannot be cleared here. However, the
  // interrupt handler cannot be allowed to fire again, so we clear TWIE to
  // disable I2C interrupts altogether.
  //
  TWCR = TWCR_DEFAULT & ~_BV(TWIE);

  task_wakeup(i2c_task);
  return;
}
