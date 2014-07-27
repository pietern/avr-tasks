#ifndef _MMA8452_H
#define _MMA8452_H

#include <stdint.h>

/* Data rates. */
#define MMA8452Q_DR2 _BV(5)
#define MMA8452Q_DR1 _BV(4)
#define MMA8452Q_DR0 _BV(3)
#define MMA8452Q_DR_800HZ (0)
#define MMA8452Q_DR_400HZ (MMA8452Q_DR0)
#define MMA8452Q_DR_200HZ (MMA8452Q_DR1)
#define MMA8452Q_DR_100HZ (MMA8452Q_DR1 | MMA8452Q_DR0)
#define MMA8452Q_DR_50HZ  (MMA8452Q_DR2)
#define MMA8452Q_DR_12HZ  (MMA8452Q_DR2 | MMA8452Q_DR0)
#define MMA8452Q_DR_6HZ   (MMA8452Q_DR2 | MMA8452Q_DR1)
#define MMA8452Q_DR_1HZ   (MMA8452Q_DR2 | MMA8452Q_DR1 | MMA8452Q_DR0)

int8_t mma8452q_configure(uint8_t dr);

int8_t mma8452q_read(int16_t *axis);

#endif
