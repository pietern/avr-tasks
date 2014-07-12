#ifndef _I2C_H
#define _I2C_H

#define I2C_FREQ 100000

void i2c_init(void);

int8_t i2c_read(uint8_t address, uint8_t *buf, uint8_t len);

int8_t i2c_write(uint8_t address, uint8_t *buf, uint8_t len);

#endif
