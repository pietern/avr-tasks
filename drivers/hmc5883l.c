#include <avr/io.h>

#include "hmc5883l.h"
#include "i2c.h"
#include "task.h"

#define HMC5883L_ADDRESS 0x1E

// Initialize the sensor by writing to its configuration and mode registers.
int8_t hmc5883l_configure(uint8_t a, uint8_t b, uint8_t m) {
  uint8_t b0[2] = { 0x0, a };
  uint8_t b1[2] = { 0x1, b };
  uint8_t b2[2] = { 0x2, m };
  int8_t rv;

  i2c_open();

  rv = i2c_write(HMC5883L_ADDRESS, b0, sizeof(b0));
  if (rv < 0) {
    goto done;
  }

  rv = i2c_write(HMC5883L_ADDRESS, b1, sizeof(b1));
  if (rv < 0) {
    goto done;
  }

  rv = i2c_write(HMC5883L_ADDRESS, b2, sizeof(b2));
  if (rv < 0) {
    goto done;
  }

done:
  i2c_close();
  return rv;
}

// Read raw values from sensor.
// Prior to calling this function, the sensor either had to be configured to
// take a single measurement (MODE_SINGLE), or to be in continuous measurement
// mode (MODE_CONTINOUS).
int8_t hmc5883l_read(int16_t axis[3]) {
  uint8_t b3[1] = { 0x03 };
  uint8_t data[6];
  int8_t rv;

  i2c_open();

  rv = i2c_write(HMC5883L_ADDRESS, b3, sizeof(b3));
  if (rv < 0) {
    goto done;
  }

  rv = i2c_read(HMC5883L_ADDRESS, data, sizeof(data));
  if (rv < 0) {
    goto done;
  }

  axis[0 /* X */] = (data[0] << 8) | data[1];
  axis[2 /* Z */] = (data[2] << 8) | data[3];
  axis[1 /* Y */] = (data[4] << 8) | data[5];

done:
  i2c_close();
  return rv;
}

// Initialize sensor struct.
void hmc5883l_init(hmc5883l_t *h) {
  int8_t i;

  for (i = 0; i < 3; i++) {
    h->cali_offset[i] = 0;
    h->cali_scale[i] = 1.0;

    h->extra_offset[i] = 0;
    h->extra_scale_pos[i] = 1.0;
    h->extra_scale_neg[i] = 1.0;
  }
}

// Execute sensor self-test to fill in the offset and scale struct fields.
int8_t hmc5883l_calibrate(hmc5883l_t *h, uint8_t gain) {
  int8_t rv;
  int16_t pos_axis[3], neg_axis[3];
  uint16_t lsb_per_gauss = 0;

  rv = hmc5883l_configure(SAMPLE_8 | MEASURE_POS_BIAS, gain, MODE_SINGLE);
  if (rv < 0) {
    return rv;
  }

  task_sleep(8);
  rv = hmc5883l_read(pos_axis);
  if (rv < 0) {
    return rv;
  }

  rv = hmc5883l_configure(SAMPLE_8 | MEASURE_NEG_BIAS, gain, MODE_SINGLE);
  if (rv < 0) {
    return rv;
  }

  task_sleep(8);
  rv = hmc5883l_read(neg_axis);
  if (rv < 0) {
    return rv;
  }

  switch(gain) {
  case GAIN_1370: lsb_per_gauss = 1370; break;
  case GAIN_1090: lsb_per_gauss = 1090; break;
  case GAIN_820: lsb_per_gauss = 820; break;
  case GAIN_660: lsb_per_gauss = 660; break;
  case GAIN_440: lsb_per_gauss = 440; break;
  case GAIN_390: lsb_per_gauss = 390; break;
  case GAIN_330: lsb_per_gauss = 330; break;
  case GAIN_230: lsb_per_gauss = 230; break;
  }

  // Both the positively and negatively biased measurements should yield the
  // same values. This offset compensates for that measurement error.
  h->cali_offset[0] = -(pos_axis[0] + neg_axis[0]) / 2;
  h->cali_offset[1] = -(pos_axis[1] + neg_axis[1]) / 2;
  h->cali_offset[2] = -(pos_axis[2] + neg_axis[2]) / 2;

  // Static scaling factor for specified gain mode
  h->cali_scale[0] = 1000.0 / (float)lsb_per_gauss;
  h->cali_scale[1] = 1000.0 / (float)lsb_per_gauss;
  h->cali_scale[2] = 1000.0 / (float)lsb_per_gauss;

  // Dynamic scaling factor for specified gain mode
  h->cali_scale[0] *= 1.160 / ((pos_axis[0] + h->cali_offset[0]) / (float)lsb_per_gauss);
  h->cali_scale[1] *= 1.160 / ((pos_axis[1] + h->cali_offset[1]) / (float)lsb_per_gauss);
  h->cali_scale[2] *= 1.080 / ((pos_axis[2] + h->cali_offset[2]) / (float)lsb_per_gauss);

  return 0;
}

// Read scaled values from sensor.
int8_t hmc5883l_read_scaled(hmc5883l_t *h, int16_t axis[3]) {
  int8_t rv;
  int8_t i;

  rv = hmc5883l_read(axis);
  if (rv < 0) {
    return rv;
  }

  for (i = 0; i < 3; i++) {
    axis[i] += h->cali_offset[i];
    axis[i] *= h->cali_scale[i];

    axis[i] += h->extra_offset[i];
    if (axis[i] > 0) {
      axis[i] *= h->extra_scale_pos[i];
    } else {
      axis[i] *= h->extra_scale_neg[i];
    }
  }

  return 0;
}
