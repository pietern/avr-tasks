#ifndef _HMC5883L_H
#define _HMC5883L_H

/*
 * From the datasheet:
 *
 * The Honeywell HMC5883L is a surface-mount, multi-chip module designed for
 * low-field magnetic sensing with a digital interface for applications such as
 * low-cost compassing and magnetometry. The HMC5883L includes our
 * state-of-the-art, high-resolution HMC118X series magneto-resistive sensors
 * plus an ASIC containing amplification, automatic degaussing strap drivers,
 * offset cancellation, and a 12-bit ADC that enables 1 to 2 degree compass
 * heading accuracy.
 *
 * Notes:
 *
 * The module can be configured to use any of 8 gain modes to increase the
 * measurement resolution at the cost of a lower range. The digital
 * measurements can be converted to milli-gauss by applying a scaling factor.
 * This scaling factor is a combination of a static factor that is documented
 * and a dynamic factor that must be determined at run-time. The static factor
 * depends on the selected gain mode and can be found in the datasheet. The
 * dynamic factor is determined by executing a self-test built into the module.
 * The self-test takes the difference between two measurements: one for
 * reference and one where an ~1.1 Gauss field is added to the existing field.
 * Because the expected measurement value for this difference is known
 * (depending on the gain mode), this measurement allows scaling to compensate
 * for environment effects (e.g. temperature).
 */

#include <stdint.h>

// Configuration register A.
#define MA1 _BV(6)
#define MA0 _BV(5)
#define DO2 _BV(4)
#define DO1 _BV(3)
#define DO0 _BV(2)
#define MS1 _BV(1)
#define MS0 _BV(0)

// Number of samples averaged (1 to 8) per measurement output.
#define SAMPLE_1 (0)
#define SAMPLE_2 (MA0)
#define SAMPLE_4 (MA1)
#define SAMPLE_8 (MA1 | MA0)

// Data Output Rate.
#define RATE_0_75_HZ (0)
#define RATE_1_5_HZ  (DO0)
#define RATE_3_HZ    (DO1)
#define RATE_7_5_HZ  (DO1 | DO0)
#define RATE_15_HZ   (DO2)
#define RATE_30_HZ   (DO2 | DO0)
#define RATE_75_HZ   (DO2 | DO1)

// Measurement Configuration.
#define MEASURE_NORMAL   (0)
#define MEASURE_POS_BIAS (MS0)
#define MEASURE_NEG_BIAS (MS1)

// Configuration register B.
#define GN2 _BV(7)
#define GN1 _BV(6)
#define GN0 _BV(5)

// Gain Configuration.
#define GAIN_1370 (0)
#define GAIN_1090 (GN0)
#define GAIN_820  (GN1)
#define GAIN_660  (GN1 | GN0)
#define GAIN_440  (GN2)
#define GAIN_390  (GN2 | GN0)
#define GAIN_330  (GN2 | GN1)
#define GAIN_230  (GN2 | GN1 | GN0)

// Mode register.
#define HS  _BV(7)
#define MD1 _BV(1)
#define MD0 _BV(0)

// Mode Select.
#define MODE_CONTINOUS (0)
#define MODE_SINGLE    (MD0)
#define MODE_IDLE      (MD1)

typedef struct hmc5883l_s hmc5883l_t;

struct hmc5883l_s {
  // Offset and scale determined by continuous self-test.
  // The result of these adjustments are values in milli-gauss.
  int16_t cali_offset[3];
  float cali_scale[3];

  // Offset and scale determined by one-time manual analysis.
  // Different scaling factors for positive and negative measurements are the
  // result of once observing a non-linearity where the negative range was
  // larger than the positive range while the axis was centered correctly.
  // This can be worked around by scaling these ranges independently.
  int16_t extra_offset[3];
  float extra_scale_pos[3];
  float extra_scale_neg[3];
};

int8_t hmc5883l_configure(uint8_t a, uint8_t b, uint8_t m);

int8_t hmc5883l_read(int16_t axis[3]);

void hmc5883l_init(hmc5883l_t *h);

int8_t hmc5883l_calibrate(hmc5883l_t *h, uint8_t gain);

int8_t hmc5883l_read_scaled(hmc5883l_t *h, int16_t axis[3]);

#endif
