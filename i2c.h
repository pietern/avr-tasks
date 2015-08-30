#ifndef _I2C_H
#define _I2C_H

#ifndef I2C_FREQ
#define I2C_FREQ 100000
#endif

// Treat arbitration loss as error.
// This implementation cannot recover from arbitration loss because the iovec
// is mutated. Recovery from arbitration loss means the interrupt handler must
// reset the iovec to its state when the transaction started.
#ifndef I2C_ARBITRATION_LOST_IS_ERROR
#define I2C_ARBITRATION_LOST_IS_ERROR 1
#endif

struct i2c_iovec_s {
  uint8_t *base;
  uint8_t len;
};

void i2c_init(void);

void i2c_open(void);

void i2c_close(void);

int8_t i2c_readv(uint8_t address, struct i2c_iovec_s *iov, uint8_t iovcnt);

int8_t i2c_writev(uint8_t address, struct i2c_iovec_s *iov, uint8_t iovcnt);

int8_t i2c_read(uint8_t address, uint8_t *buf, uint8_t len);

int8_t i2c_write(uint8_t address, uint8_t *buf, uint8_t len);

int8_t i2c_read_from(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t len);

int8_t i2c_write_to(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t len);

#endif
