#include <avr/io.h>

#include "i2c.h"
#include "mma8452q.h"

/* Slave address selection bit */
#define SA0 1
#define MMA8452Q_ADDRESS (0x1C | (SA0))

/* Data Status Register */
#define STATUS 0x00

/* Data Registers */
#define OUT_X_MSB 0x01
#define OUT_X_LSB 0x02
#define OUT_Y_MSB 0x03
#define OUT_Y_LSB 0x04
#define OUT_Z_MSB 0x05
#define OUT_Z_LSB 0x06

/* System Mode Register */
#define SYSMOD 0x0B

/* System Interrupt Status Register */
#define INT_SOURCE 0x0C

/* Device ID Register */
#define WHO_AM_I 0x0D

#define XYZ_DATA_CFG     0x0E

#define HPF_OUT _BV(4)
#define FS1     _BV(1)
#define FS0     _BV(0)

#define RANGE_2G (0)
#define RANGE_4G (FS0)
#define RANGE_8G (FS1)

#define HP_FILTER_CUTOFF 0x0F

#define PULSE_HPF_BYP _BV(5)
#define PULSE_LPF_EN  _BV(4)
#define SEL1          _BV(1)
#define SEL0          _BV(0)

/* Portrait/ Landscape Embedded Function Registers */
#define PL_STATUS   0x10
#define PL_CFG      0x11
#define PL_COUNT    0x12
#define PL_BF_ZCOMP 0x13
#define PL_THS_REG  0x14

/* Motion and Freefall Embedded Function Registers */
#define FF_MT_CFG   0x15
#define FF_MT_SRC   0x16
#define FF_MT_THS   0x17
#define FF_MT_COUNT 0x18

/* Transient (HPF) Acceleration Detection */
#define TRANSIENT_CFG   0x1D
#define TRANSIENT_SRC   0x1E
#define TRANSIENT_THS   0x1F
#define TRANSIENT_COUNT 0x20

/* Single, Double, and Directional Tap Detection Registers */
#define PULSE_CFG  0x21
#define PULSE_SRC  0x22
#define PULSE_THSX 0x23
#define PULSE_THSY 0x24
#define PULSE_THSZ 0x25
#define PULSE_TMLT 0x26
#define PULSE_LTCY 0x27
#define PULSE_WIND 0x28

/* Auto-WAKE/SLEEP Detection */
#define ASLP_COUNT 0x29

/* Control Registers */
#define CTRL_REG1 0x2A
#define CTRL_REG2 0x2B
#define CTRL_REG3 0x2C
#define CTRL_REG4 0x2D
#define CTRL_REG5 0x2E

/* User Offset Correction Registers */
#define OFF_X 0x2F
#define OFF_Y 0x30
#define OFF_Z 0x31

/* CTRL_REG1 */
#define ASLP_RATE1 _BV(7)
#define ASLP_RATE0 _BV(6)
#define DR2        _BV(5)
#define DR1        _BV(4)
#define DR0        _BV(3)
#define LNOISE     _BV(2)
#define F_READ     _BV(1)
#define ACTIVE     _BV(0)

#define ASLP_RATE_50HZ (0)
#define ASLP_RATE_12HZ (ASLP_RATE0)
#define ASLP_RATE_6HZ  (ASLP_RATE1)
#define ASLP_RATE_1HZ  (ASLP_RATE1 | ASLP_RATE0)

#define DR_800HZ (0)
#define DR_400HZ (DR0)
#define DR_200HZ (DR1)
#define DR_100HZ (DR1 | DR0)
#define DR_50HZ  (DR2)
#define DR_12HZ  (DR2 | DR0)
#define DR_6HZ   (DR2 | DR1)
#define DR_1HZ   (DR2 | DR1 | DR0)

/* CTRL_REG2 */
#define ST     _BV(7)
#define RST    _BV(6)
#define SMODS1 _BV(4)
#define SMODS0 _BV(3)
#define SLPE   _BV(2)
#define MODS1  _BV(1)
#define MODS0  _BV(0)

/* CTRL_REG3 */
#define WAKE_TRANS  _BV(6)
#define WAKE_LNDPRT _BV(5)
#define WAKE_PULSE  _BV(4)
#define WAKE_FF_MT  _BV(3)
#define IPOL        _BV(1)
#define PP_OD       _BV(0)

/* CTRL_REG4 */
#define INT_EN_ASLP   _BV(7)
#define INT_EN_TRANS  _BV(5)
#define INT_EN_LNDPRT _BV(4)
#define INT_EN_PULSE  _BV(3)
#define INT_EN_FF_MT  _BV(2)
#define INT_EN_DRDY   _BV(0)

/* CTRL_REG5 */
#define INT_CFG_ASLP   _BV(7)
#define INT_CFG_TRANS  _BV(5)
#define INT_CFG_LNDPRT _BV(4)
#define INT_CFG_PULSE  _BV(3)
#define INT_CFG_FF_MT  _BV(2)
#define INT_CFG_DRDY   _BV(0)

// Public initialization function (only sets data rate).
int8_t mma8452q_configure(uint8_t d) {
  int8_t rv;

  i2c_open();

  // Switch sensor to STANDBY mode.
  {
    uint8_t ctrl[6] = { CTRL_REG1, 0, 0, 0, 0, 0 };
    rv = i2c_write(MMA8452Q_ADDRESS, ctrl, 6);
    if (rv < 0) {
      goto done;
    }
  }

  // Set resolution.
  // This only takes effect if the sensor is in STANDBY mode.
  {
    uint8_t hpf[] = { XYZ_DATA_CFG, RANGE_2G, 0 };

    rv = i2c_write(MMA8452Q_ADDRESS, hpf, sizeof(hpf));
    if (rv < 0) {
      goto done;
    }
  }

  // Switch sensor to ACTIVE mode.
  {
    uint8_t ctrl[6] = { CTRL_REG1, d | ACTIVE, 0, 0, 0, 0 };
    rv = i2c_write(MMA8452Q_ADDRESS, ctrl, 6);
    if (rv < 0) {
      goto done;
    }
  }

done:
  i2c_close();
  return rv;
}

int8_t mma8452q_read(int16_t axis[3]) {
  uint8_t r = OUT_X_MSB;
  uint8_t data[6];
  int8_t rv, i;

  i2c_open();

  rv = i2c_write(MMA8452Q_ADDRESS, &r, 1);
  if (rv < 0) {
    goto error;
  }

  rv = i2c_read(MMA8452Q_ADDRESS, data, 6);
  if (rv < 0) {
    goto error;
  }

  // Every axis has two registers holding a measurement value MSB and LSB.
  // The compiler uses little endian representation, so we need to swap these
  // bytes before we can interpret the value as an 16-bit integer.
  for (i = 0; i < 6; i += 2) {
    r = data[i];
    data[i] = data[i+1];
    data[i+1] = r;
  }

  // Drop lowest 4 bits (measurements are 12-bit values).
  for (i = 0; i < 3; i++) {
    axis[i] = ((int16_t *)data)[i] >> 4;
  }

error:
  i2c_close();
  return rv;
}
