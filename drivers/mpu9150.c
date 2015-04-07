#include <avr/io.h>
#include <math.h>

#include "mpu9150.h"
#include "i2c.h"
#include "task.h"

#define AK8975_ADDR 0x0C
#define MPU9150_ADDR 0x68

// Registers
#define WIA 0x00
#define INFO 0x01
#define ST1 0x02
#define HXL 0x03
#define HXH 0x04
#define HYL 0x05
#define HYH 0x06
#define HZL 0x07
#define HZH 0x08
#define ST2 0x09
#define CNTL 0x0A
#define RSV 0x0B
#define ASTC 0x0C
#define TS1 0x0D
#define TS2 0x0E
#define I2CDIS 0x0F
#define ASAX 0x10
#define ASAY 0x11
#define ASAZ 0x12

// DRDY field in ST1 register
#define DRDY _BV(0)

// MODE fields in CNTL register
#define POWER_DOWN_MODE 0x0
#define SINGLE_MEASUREMENT_MODE 0x1
#define SELF_TEST_MODE 0x8
#define FUSE_ROM_ACCESS_MODE 0xf

// SELF field in ASTC register
#define GENERATE_MAGNETIC_FIELD_FOR_SELF_TEST _BV(6)

// Gyroscope and accelerometer registers
#define SELF_TEST_X 0x0D
#define SELF_TEST_Y 0x0E
#define SELF_TEST_Z 0x0F
#define SELF_TEST_A 0x10
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define FF_THR 0x1D
#define FF_DUR 0x1E
#define MOT_THR 0x1F
#define MOT_DUR 0x20
#define ZRMOT_THR 0x21
#define ZRMOT_DUR 0x22
#define FIFO_EN 0x23
#define I2C_MST_CTRL 0x24
#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_CTRL 0x27
#define I2C_SLV1_ADDR 0x28
#define I2C_SLV1_REG 0x29
#define I2C_SLV1_CTRL 0x2A
#define I2C_SLV2_ADDR 0x2B
#define I2C_SLV2_REG 0x2C
#define I2C_SLV2_CTRL 0x2D
#define I2C_SLV3_ADDR 0x2E
#define I2C_SLV3_REG 0x2F
#define I2C_SLV3_CTRL 0x30
#define I2C_SLV4_ADDR 0x31
#define I2C_SLV4_REG 0x32
#define I2C_SLV4_DO 0x33
#define I2C_SLV4_CTRL 0x34
#define I2C_SLV4_DI 0x35
#define I2C_MST_STATUS 0x36
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define INT_STATUS 0x3A
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO 0x63
#define I2C_SLV1_DO 0x64
#define I2C_SLV2_DO 0x65
#define I2C_SLV3_DO 0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET 0x68
#define MOT_DETECT_CTRL 0x69
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define FIFO_COUNTH 0x72
#define FIFO_COUNTL 0x73
#define FIFO_R_W 0x74
#define WHO_AM_I 0x75

// Fields for GYRO_CONFIG
#define XG_ST _BV(7)
#define YG_ST _BV(6)
#define ZG_ST _BV(5)
#define GYRO_250_DPS (0 * _BV(4) | 0 * _BV(3))
#define GYRO_500_DPS (0 * _BV(4) | 1 * _BV(3))
#define GYRO_1000_DPS (1 * _BV(4) | 0 * _BV(3))
#define GYRO_2000_DPS (1 * _BV(4) | 1 * _BV(3))

// Fields for ACCEL_CONFIG
#define XA_ST _BV(7)
#define YA_ST _BV(6)
#define ZA_ST _BV(5)
#define ACCEL_2G  ((0 << 3) & 0x18)
#define ACCEL_4G  ((1 << 3) & 0x18)
#define ACCEL_8G  ((2 << 3) & 0x18)
#define ACCEL_16G ((3 << 3) & 0x18)
#define ACCEL_HPF_RESET  (0 & 0x7)
#define ACCEL_HPF_5HZ    (1 & 0x7)
#define ACCEL_HPF_2_5HZ  (2 & 0x7)
#define ACCEL_HPF_1_25HZ (3 & 0x7)
#define ACCEL_HPF_0_63HZ (4 & 0x7)
#define ACCEL_HPF_HOLD   (7 & 0x7)

// Fields for FIFO_EN
#define TEMP_FIFO_EN _BV(7)
#define XG_FIFO_EN _BV(6)
#define YG_FIFO_EN _BV(5)
#define ZG_FIFO_EN _BV(4)
#define ACCEL_FIFO_EN _BV(3)
#define SLV2_FIFO_EN _BV(2)
#define SLV1_FIFO_EN _BV(1)
#define SLV0_FIFO_EN _BV(0)

// Fields for I2C_MST_CTRL
#define MULT_MST_EN _BV(7)
#define WAIT_FOR_ES _BV(6)
#define SLV_3_FIFO_EN _BV(5)
#define I2C_MST_P_NSR _BV(4)
#define I2C_MST_CLK_348KHZ 0
#define I2C_MST_CLK_333KHZ 1
#define I2C_MST_CLK_320KHZ 2
#define I2C_MST_CLK_308KHZ 3
#define I2C_MST_CLK_296KHZ 4
#define I2C_MST_CLK_286KHZ 5
#define I2C_MST_CLK_276KHZ 6
#define I2C_MST_CLK_267KHZ 7
#define I2C_MST_CLK_258KHZ 8
#define I2C_MST_CLK_500KHZ 9
#define I2C_MST_CLK_471KHZ 10
#define I2C_MST_CLK_444KHZ 11
#define I2C_MST_CLK_421KHZ 12
#define I2C_MST_CLK_400KHZ 13
#define I2C_MST_CLK_381KHZ 14
#define I2C_MST_CLK_364KHZ 15

// Fields for I2C_SLV4_CTRL
#define I2C_SLV4_EN _BV(7)
#define I2C_SLV4_INT_EN _BV(6)
#define I2C_SLV4_REG_DIS _BV(6)

// Fields for I2C_MST_STATUS
#define PASS_THROUGH _BV(7)
#define I2C_SLV4_DONE _BV(6)
#define I2C_LOST_ARB _BV(5)
#define I2C_SLV4_NACK _BV(4)
#define I2C_SLV3_NACK _BV(3)
#define I2C_SLV2_NACK _BV(2)
#define I2C_SLV1_NACK _BV(1)
#define I2C_SLV0_NACK _BV(0)

// Fields for INT_PIN_CFG
#define INT_LEVEL _BV(7)
#define INT_OPEN _BV(6)
#define LATCH_INT_EN _BV(5)
#define INT_RD_CLEAR _BV(4)
#define FSYNC_INT_LEVEL _BV(3)
#define FSYNC_INT_EN _BV(2)
#define I2C_BYPASS_EN _BV(1)
#define CLKOUT_EN _BV(0)

// Fields for INT_ENABLE
#define FF_EN _BV(7)
#define MOT_EN _BV(6)
#define ZMOT_EN _BV(5)
#define FIFO_OFLOW_EN _BV(4)
#define I2C_MST_INT_EN _BV(3)
#define DATA_RDY_EN _BV(0)

// Fields for USER_CTRL
#define USER_CTRL_FIFO_EN _BV(6)
#define USER_CTRL_I2C_MST_EN _BV(5)
#define USER_CTRL_I2C_IF_DIS _BV(4)
#define USER_CTRL_FIFO_RESET _BV(2)
#define USER_CTRL_I2C_MST_RESET _BV(1)
#define USER_CTRL_SIG_COND_RESET _BV(0)

// Fields for PWR_MGMT_1
#define DEVICE_RESET _BV(7)
#define SLEEP _BV(6)
#define CYCLE  _BV(5)
#define TEMP_DIS _BV(3)
#define CLKSEL_INT 0
#define CLKSEL_PLL_XG 1
#define CLKSEL_PLL_YG 2
#define CLKSEL_PLL_ZG 3
#define CLKSEL_EXT_32KHZ 4
#define CLKSEL_EXT_19MHZ 5
#define CLKSEL_RESERVED 6
#define CLKSEL_STOP 7

// Fields for PWR_MGMT_2
#define LP_WAKE_CTRL_1HZ (0x0 << 6)
#define LP_WAKE_CTRL_5HZ (0x1 << 6)
#define LP_WAKE_CTRL_20HZ (0x2 << 6)
#define LP_WAKE_CTRL_40HZ (0x3 << 6)
#define STBY_XA _BV(5)
#define STBY_YA _BV(4)
#define STBY_ZA _BV(3)
#define STBY_XG _BV(2)
#define STBY_YG _BV(1)
#define STBY_ZG _BV(0)

static int8_t mpu9150__write_to(mpu9150_t *m, uint8_t reg, uint8_t *buf, uint8_t len) {
  int8_t rv;

  i2c_open();
  rv = i2c_write_to(MPU9150_ADDR, reg, buf, len);
  i2c_close();

  return rv;
}

static int8_t mpu9150__read_from(mpu9150_t *m, uint8_t reg, uint8_t *buf, uint8_t len) {
  int8_t rv;

  i2c_open();
  rv = i2c_read_from(MPU9150_ADDR, reg, buf, len);
  i2c_close();

  return rv;
}

static int8_t mpu9150__mag_write(mpu9150_t *m, uint8_t reg, uint8_t byte) {
  uint8_t v[4];
  int8_t rv;

  v[0] = AK8975_ADDR;
  v[1] = reg;
  v[2] = byte;
  v[3] = I2C_SLV4_EN | I2C_SLV4_INT_EN;
  rv = mpu9150__write_to(m, I2C_SLV4_ADDR, v, sizeof(v));
  if (rv < 0) {
    return rv;
  }

  // Wait for transaction to complete
  for (;;) {
    rv = mpu9150__read_from(m, I2C_MST_STATUS, v, sizeof(v[0]));
    if (rv < 0) {
      return rv;
    }

    if (v[0] & I2C_SLV4_DONE) {
      break;
    }

    task_sleep(1);
  }

  return 0;
}

static int8_t mpu9150__mag_read(mpu9150_t *m, uint8_t reg, uint8_t *b) {
  uint8_t v[4];
  int8_t rv;

  v[0] = _BV(7) | AK8975_ADDR;
  v[1] = reg;
  v[2] = 0x0;
  v[3] = I2C_SLV4_EN | I2C_SLV4_INT_EN;
  rv = mpu9150__write_to(m, I2C_SLV4_ADDR, v, sizeof(v));
  if (rv < 0) {
    return rv;
  }

  // Wait for transaction to complete
  for (;;) {
    rv = mpu9150__read_from(m, I2C_MST_STATUS, v, sizeof(v[0]));
    if (rv < 0) {
      return rv;
    }

    if (v[0] & I2C_SLV4_DONE) {
      break;
    }

    task_sleep(1);
  }

  // Read byte from I2C_SLV4_DI
  rv = mpu9150__read_from(m, I2C_SLV4_DI, b, sizeof(*b));
  if (rv < 0) {
    return rv;
  }

  return 0;
}

static int8_t mpu9150__mag_load_sensitivity_adjustment(mpu9150_t *m) {
  int8_t rv;
  uint8_t v;
  int8_t i;

  rv = mpu9150__mag_write(m, CNTL, FUSE_ROM_ACCESS_MODE);
  if (rv < 0) {
    return rv;
  }

  for (i = 0; i < 3; i++) {
    rv = mpu9150__mag_read(m, ASAX + i, &v);
    if (rv < 0) {
      return rv;
    }

    // See AK8975 datasheet, section 8.3.11.
    m->mag_adj[i] = ((v - 128) / 256.0f) + 1.0f;
  }

  rv = mpu9150__mag_write(m, CNTL, POWER_DOWN_MODE);
  if (rv < 0) {
    return rv;
  }

  return 0;
}

static int8_t mpu9150__core_init(mpu9150_t *m) {
  uint8_t v;
  int8_t rv;

  i2c_open();

  // CONFIG
  // - Enable digital low pass filter (output 40Hz)
  v = 0x3;
  rv = i2c_write_to(MPU9150_ADDR, CONFIG, &v, sizeof(v));
  if (rv < 0) {
    goto done;
  }

  // USER_CTRL
  v = USER_CTRL_I2C_MST_EN;
  rv = i2c_write_to(MPU9150_ADDR, USER_CTRL, &v, sizeof(v));
  if (rv < 0) {
    goto done;
  }

  // PWR_MGMT_1
  v = CLKSEL_PLL_XG;
  rv = i2c_write_to(MPU9150_ADDR, PWR_MGMT_1, &v, sizeof(v));
  if (rv < 0) {
    goto done;
  }

  // PWR_MGMT_2
  v = 0;
  rv = i2c_write_to(MPU9150_ADDR, PWR_MGMT_2, &v, sizeof(v));
  if (rv < 0) {
    goto done;
  }

  // I2C_MST_CTRL
  v = WAIT_FOR_ES;
  rv = i2c_write_to(MPU9150_ADDR, I2C_MST_CTRL, &v, sizeof(v));
  if (rv < 0) {
    goto done;
  }

  // INT_PIN_CFG:
  // - Interrupt pin is active high
  // - Interrupt pin is push-pull
  // - Interrupt pin emits 50us long pulse
  // - Interrupt status bits are cleared by reading INT_STATUS
  // - FSYNC is active high
  // - FSYNC is disabled
  // - I2C bypass is disabled
  // - Reference clock output is disabled
  v = 0;
  rv = i2c_write_to(MPU9150_ADDR, INT_PIN_CFG, &v, sizeof(v));
  if (rv < 0) {
    goto done;
  }

  // INT_ENABLE
  v = FIFO_OFLOW_EN | I2C_MST_INT_EN | DATA_RDY_EN;
  rv = i2c_write_to(MPU9150_ADDR, INT_ENABLE, &v, sizeof(v));
  if (rv < 0) {
    goto done;
  }

done:
  i2c_close();

  return rv;
}

static int8_t mpu9150__mag_init(mpu9150_t *m) {
  int8_t rv;

  rv = mpu9150__mag_load_sensitivity_adjustment(m);
  if (rv < 0) {
    return rv;
  }

  return 0;
}

int8_t mpu9150_init(mpu9150_t *m) {
  int8_t rv;

  rv = mpu9150__core_init(m);
  if (rv < 0) {
    return rv;
  }

  rv = mpu9150__mag_init(m);
  if (rv < 0) {
    return rv;
  }

  return 0;
}

// mpu9150__mag_wait_read waits until the AK8975's DRDY bit is read,
// then reads the measurement registers, and performs sensitivity
// adjustment before returning.
static int8_t mpu9150__mag_wait_read(mpu9150_t *m, int16_t r[3]) {
  int8_t rv;
  uint8_t v;

  // Wait for DRDY bit of ST1 register
  for (;;) {
    rv = mpu9150__mag_read(m, ST1, &v);
    if (rv < 0) {
      goto done;
    }

    if (v & DRDY) {
      break;
    }
  }

  // Read sensor data
  uint8_t *ptr = ((uint8_t *) r);
  for (v = 0; v < 6; v++) {
    rv = mpu9150__mag_read(m, HXL + v, ptr + v);
    if (rv < 0) {
      goto done;
    }
  }

  // Factor in sensitivity adjustment
  r[0] *= m->mag_adj[0];
  r[1] *= m->mag_adj[1];
  r[2] *= m->mag_adj[2];

done:
  return rv;
}

// mpu9150_mag_self_test executes self test sequence
// as defined in the datasheet for AK8975.
int8_t mpu9150_mag_self_test(mpu9150_t *m, int16_t r[3]) {
  int8_t rv;

  // Set power down mode
  rv = mpu9150__mag_write(m, CNTL, POWER_DOWN_MODE);
  if (rv < 0) {
    return rv;
  }

  // Write 1 to SELF bit of ASTC register
  rv = mpu9150__mag_write(m, ASTC, GENERATE_MAGNETIC_FIELD_FOR_SELF_TEST);
  if (rv < 0) {
    return rv;
  }

  // Set self test mode
  rv = mpu9150__mag_write(m, CNTL, SELF_TEST_MODE);
  if (rv < 0) {
    return rv;
  }

  // Wait for data
  rv = mpu9150__mag_wait_read(m, r);
  if (rv < 0) {
    return rv;
  }

  // Write 0 to SELF bit of ASTC register
  rv = mpu9150__mag_write(m, ASTC, 0x0);
  if (rv < 0) {
    return rv;
  }

  return 0;
}

// mpu9150_mag_single_measurement executes single measurement and
// performs sensisivity adjustment before returning.
int8_t mpu9150_mag_single_measurement(mpu9150_t *m, int16_t r[3]) {
  int8_t rv;

  // Set single measurement mode
  rv = mpu9150__mag_write(m, CNTL, SINGLE_MEASUREMENT_MODE);
  if (rv < 0) {
    return rv;
  }

  // Wait for data
  rv = mpu9150__mag_wait_read(m, r);
  if (rv < 0) {
    return rv;
  }

  return 0;
}

static void mpu9150__swap_endian(int16_t *r) {
  *r = ((*r >> 8) & 0xff) | (*r << 8);
}

// mpu9150__read_values reads a single gyroscope or accelerometer
// measurement and swaps the byte order before returning.
static int8_t mpu9150__read_values(mpu9150_t *m, uint8_t reg, int16_t raw[3]) {
  int8_t rv;

  rv = mpu9150__read_from(m, reg, (uint8_t *) raw, 6);
  if (rv < 0) {
    return rv;
  }

  mpu9150__swap_endian(&raw[0]);
  mpu9150__swap_endian(&raw[1]);
  mpu9150__swap_endian(&raw[2]);

  return 0;
}

// mpu9150__write_values writes a set of values to the specified register
// and swaps the byte order before doing so.
static int8_t mpu9150__write_values(mpu9150_t *m, uint8_t reg, int16_t raw[3]) {
  int8_t rv;

  mpu9150__swap_endian(&raw[0]);
  mpu9150__swap_endian(&raw[1]);
  mpu9150__swap_endian(&raw[2]);

  rv = mpu9150__write_to(m, reg, (uint8_t *) raw, 6);
  if (rv < 0) {
    return rv;
  }

  return 0;
}

// mpu9150_gyro_get_factory_trim computes factory trim.
//
// See MPU9150 register map datasheet, registers 13 to 16, under:
// "Obtaining the Gyroscope Factory Trim (FT) Value".
//
int8_t mpu9150_gyro_get_factory_trim(mpu9150_t *m, float r[3]) {
  int8_t rv;
  uint8_t ft[3];

  // Read factory trim inputs
  rv = mpu9150__read_from(m, SELF_TEST_X, ft, sizeof(ft));
  if (rv < 0) {
    return rv;
  }

  // The factory trim values are 5 bit integers
  ft[0] &= 0x1f;
  ft[1] &= 0x1f;
  ft[2] &= 0x1f;

  // Compute factory trim
  r[0] =  25.0f * 131.0f * pow(1.046f, ft[0]-1.0f);
  r[1] = -25.0f * 131.0f * pow(1.046f, ft[1]-1.0f);
  r[2] =  25.0f * 131.0f * pow(1.046f, ft[2]-1.0f);

  return 0;
}

int8_t mpu9150_gyro_self_test(mpu9150_t *m, float r[3]) {
  int8_t rv;
  uint8_t u;
  uint8_t v;
  float ft[3];
  int16_t str[6];
  int8_t i;

  // Read original range
  rv = mpu9150__read_from(m, GYRO_CONFIG, &u, sizeof(u));
  if (rv < 0) {
    return rv;
  }

  // Mask result to get original range
  u &= (_BV(4) | _BV(3));

  // Enable self test on three axes and set range to +/- 250 dps
  v = XG_ST | YG_ST | ZG_ST | GYRO_250_DPS;
  rv = mpu9150__write_to(m, GYRO_CONFIG, &v, sizeof(v));
  if (rv < 0) {
    return rv;
  }

  // Get factory trim values for gyro
  rv = mpu9150_gyro_get_factory_trim(m, ft);
  if (rv < 0) {
    return rv;
  }

  // Wait for measurement
  task_sleep(1);

  // Read gyro measurement
  rv = mpu9150__read_values(m, GYRO_XOUT_H, str);
  if (rv < 0) {
    return rv;
  }

  // Disable self test on three axes and restore range
  rv = mpu9150__write_to(m, GYRO_CONFIG, &u, sizeof(u));
  if (rv < 0) {
    return rv;
  }

  // Compute difference from factory trim
  for (i = 0; i < 3; i++) {
    r[i] = (str[i] - ft[i]) / ft[i];
  }

  return 0;
}

int8_t mpu9150_gyro_single_measurement(mpu9150_t *m, int16_t r[3]) {
  int8_t rv;

  rv = mpu9150__read_values(m, GYRO_XOUT_H, r);
  if (rv < 0) {
    return rv;
  }

  return 0;
}

int8_t mpu9150_gyro_read_offset(mpu9150_t *m, int16_t r[3]) {
  return mpu9150__read_values(m, 0x13, r);
}

int8_t mpu9150_gyro_write_offset(mpu9150_t *m, int16_t r[3]) {
  return mpu9150__write_values(m, 0x13, r);
}

// mpu9150_accel_get_factory_trim computes factory trim.
//
// See MPU9150 register map datasheet, registers 13 to 16, under:
// "Obtaining the Accelerometer Factory Trim (FT) Value".
//
int8_t mpu9150_accel_get_factory_trim(mpu9150_t *m, float r[3]) {
  int8_t rv;
  uint8_t ft[4];

  // Read factory trim inputs
  rv = mpu9150__read_from(m, SELF_TEST_X, ft, sizeof(ft));
  if (rv < 0) {
    return rv;
  }

  // The factory trim values are 5 bit integers
  ft[0] = (((ft[0] & 0xe0) >> 3) | ((ft[3] >> 4) & 0x3));
  ft[1] = (((ft[1] & 0xe0) >> 3) | ((ft[3] >> 2) & 0x3));
  ft[2] = (((ft[2] & 0xe0) >> 3) | ((ft[3] >> 0) & 0x3));

  // Compute factory trim
  float tmp = 0.92f / 0.34f;
  r[0] = 4096.0f * 0.34f * pow(tmp, (ft[0] - 1.0f) / 30.0f);
  r[1] = 4096.0f * 0.34f * pow(tmp, (ft[1] - 1.0f) / 30.0f);
  r[2] = 4096.0f * 0.34f * pow(tmp, (ft[2] - 1.0f) / 30.0f);

  return 0;
}

int8_t mpu9150_accel_self_test(mpu9150_t *m, float r[3]) {
  int8_t rv;
  uint8_t u;
  uint8_t v;
  float ft[3];
  int16_t str[3];
  int8_t i;

  // Read original range
  rv = mpu9150__read_from(m, ACCEL_CONFIG, &u, sizeof(u));
  if (rv < 0) {
    return rv;
  }

  // Mask result to get original range
  u &= (_BV(4) | _BV(3));

  // Enable self test on three axes and set range to +/- 8g
  v = XA_ST | YA_ST | ZA_ST | ACCEL_8G;
  rv = mpu9150__write_to(m, ACCEL_CONFIG, &v, sizeof(v));
  if (rv < 0) {
    return rv;
  }

  // Get factory trim values for accelerometer
  rv = mpu9150_accel_get_factory_trim(m, ft);
  if (rv < 0) {
    return rv;
  }

  // Wait for measurement
  task_sleep(1);

  // Read accelerometer measurement
  rv = mpu9150__read_values(m, ACCEL_XOUT_H, str);
  if (rv < 0) {
    return rv;
  }

  // Disable self test on three axes and restore range
  rv = mpu9150__write_to(m, ACCEL_CONFIG, &u, sizeof(u));
  if (rv < 0) {
    return rv;
  }

  // Compute difference from factory trim
  for (i = 0; i < 3; i++) {
    r[i] = (str[i] - ft[i]) / ft[i];
  }

  return 0;
}

int8_t mpu9150_accel_single_measurement(mpu9150_t *m, int16_t r[3]) {
  int8_t rv;

  rv = mpu9150__read_values(m, ACCEL_XOUT_H, r);
  if (rv < 0) {
    return rv;
  }

  return 0;
}

int8_t mpu9150_accel_read_offset(mpu9150_t *m, int16_t r[3]) {
  return mpu9150__read_values(m, 0x06, r);
}

int8_t mpu9150_accel_write_offset(mpu9150_t *m, int16_t r[3]) {
  return mpu9150__write_values(m, 0x06, r);
}

int8_t mpu9150_temp_single_measurement(mpu9150_t *m, float *degc) {
  int8_t rv;
  int16_t temp;

  rv = mpu9150__read_from(m, TEMP_OUT_H, (uint8_t *) &temp, 2);
  if (rv < 0) {
    return rv;
  }

  mpu9150__swap_endian(&temp);
  *degc = (temp / 340.0f) + 35.0f;

  return 0;
}

#if MPU9150_FIFO
int8_t mpu9150_fifo_init(mpu9150_t *m) {
  uint8_t v;
  int8_t rv;

  i2c_open();

  // SMPLRT_DIV
  // With low pass filter enabled (below), divide 1KHz by 10 to get 100Hz
  v = 9;
  rv = i2c_write_to(MPU9150_ADDR, SMPLRT_DIV, &v, sizeof(v));
  if (rv < 0) {
    goto done;
  }

  // CONFIG
  // - Enable digital low pass filter (output 10Hz)
  v = 0x5;
  rv = i2c_write_to(MPU9150_ADDR, CONFIG, &v, sizeof(v));
  if (rv < 0) {
    goto done;
  }

  // USER_CTRL
  v = 0
    | USER_CTRL_FIFO_EN
    | USER_CTRL_FIFO_RESET
    | USER_CTRL_I2C_MST_EN
    | USER_CTRL_I2C_MST_RESET
    ;
  rv = i2c_write_to(MPU9150_ADDR, USER_CTRL, &v, sizeof(v));
  if (rv < 0) {
    goto done;
  }

  // PWR_MGMT_1
  v = CLKSEL_PLL_XG;
  rv = i2c_write_to(MPU9150_ADDR, PWR_MGMT_1, &v, sizeof(v));
  if (rv < 0) {
    goto done;
  }

  // PWR_MGMT_2
  v = 0;
  rv = i2c_write_to(MPU9150_ADDR, PWR_MGMT_2, &v, sizeof(v));
  if (rv < 0) {
    goto done;
  }

  // FIFO_EN
  v = 0
#if MPU9150_FIFO_TEMP
    | TEMP_FIFO_EN
#endif
#if MPU9150_FIFO_GYRO
    | XG_FIFO_EN
    | YG_FIFO_EN
    | ZG_FIFO_EN
#endif
#if MPU9150_FIFO_ACCEL
    | ACCEL_FIFO_EN
#endif
    /*| SLV0_FIFO_EN*/
    ;
  rv = i2c_write_to(MPU9150_ADDR, FIFO_EN, &v, sizeof(v));
  if (rv < 0) {
    goto done;
  }

  // I2C_MST_CTRL
  v = 0
    | WAIT_FOR_ES
    | I2C_MST_CLK_400KHZ
    ;
  rv = i2c_write_to(MPU9150_ADDR, I2C_MST_CTRL, &v, sizeof(v));
  if (rv < 0) {
    goto done;
  }

  // INT_PIN_CFG:
  // - Interrupt pin is active high
  // - Interrupt pin is push-pull
  // - Interrupt pin emits 50us long pulse
  // - Interrupt status bits are cleared by reading INT_STATUS
  // - FSYNC is active high
  // - FSYNC is disabled
  // - I2C bypass is disabled
  // - Reference clock output is disabled
  v = 0;
  rv = i2c_write_to(MPU9150_ADDR, INT_PIN_CFG, &v, sizeof(v));
  if (rv < 0) {
    goto done;
  }

  // INT_ENABLE
  v = FIFO_OFLOW_EN | I2C_MST_INT_EN | DATA_RDY_EN;
  rv = i2c_write_to(MPU9150_ADDR, INT_ENABLE, &v, sizeof(v));
  if (rv < 0) {
    goto done;
  }

done:
  i2c_close();

  return rv;
}

int8_t mpu9150_fifo_read_len(mpu9150_t *m, int16_t *len) {
  int8_t rv;

  rv = mpu9150__read_from(m, FIFO_COUNTH, (uint8_t *) len, 2);
  if (rv < 0) {
    goto done;
  }

  mpu9150__swap_endian(len);

done:
  return rv;
}

int8_t mpu9150_fifo_read(mpu9150_t *m, mpu9150_fifo_data_t *data) {
  uint8_t iovcnt;
  struct i2c_iovec_s iov[] = {
    { } // Empty initial to allow for prefixing separator below
#if MPU9150_FIFO_ACCEL
    , { .base = (uint8_t *)(data->accel), .len = 6 }
#endif
#if MPU9150_FIFO_TEMP
    , { .base = (uint8_t *)(&data->temp), .len = 2 }
#endif
#if MPU9150_FIFO_GYRO
    , { .base = (uint8_t *)(data->gyro), .len = 6 }
#endif
  };

  int8_t rv;
  int8_t i;

  i2c_open();

  iovcnt = (sizeof(iov) / sizeof(iov[0]));
  rv = i2c_readv_from(MPU9150_ADDR, FIFO_R_W, &iov[1], iovcnt-1);
  if (rv < 0) {
    goto done;
  }

#if MPU9150_FIFO_ACCEL
  for (i = 0; i < 3; i++) {
    mpu9150__swap_endian(&data->accel[i]);
  }
#endif

#if MPU9150_FIFO_TEMP
  mpu9150__swap_endian(&data->temp);
#endif

#if MPU9150_FIFO_GYRO
  for (i = 0; i < 3; i++) {
    mpu9150__swap_endian(&data->gyro[i]);
  }
#endif

done:
  i2c_close();

  return rv;
}
#endif
