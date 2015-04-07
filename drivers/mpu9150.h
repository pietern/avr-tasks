#ifndef _MPU9150_H
#define _MPU9150_H

#include <avr/io.h>
#include <stdio.h>

// Enable FIFO mode by default
#ifndef MPU9150_FIFO
#define MPU9150_FIFO 1
#endif

#if MPU9150_FIFO
#ifndef MPU9150_FIFO_MAG
#define MPU9150_FIFO_MAG 0
#endif
#ifndef MPU9150_FIFO_GYRO
#define MPU9150_FIFO_GYRO 1
#endif
#ifndef MPU9150_FIFO_ACCEL
#define MPU9150_FIFO_ACCEL 1
#endif
#ifndef MPU9150_FIFO_TEMP
#define MPU9150_FIFO_TEMP 1
#endif
#endif

#if MPU9150_FIFO
typedef struct mpu9150_fifo_data_s mpu9150_fifo_data_t;
struct mpu9150_fifo_data_s {
#if MPU9150_FIFO_MAG
  int16_t mag[3];
#endif
#if MPU9150_FIFO_GYRO
  int16_t gyro[3];
#endif
#if MPU9150_FIFO_ACCEL
  int16_t accel[3];
#endif
#if MPU9150_FIFO_TEMP
  int16_t temp;
#endif
};
#endif

#define MPU9150_ACCEL_2G  (0 * _BV(4) | 0 * _BV(3))
#define MPU9150_ACCEL_4G  (0 * _BV(4) | 1 * _BV(3))
#define MPU9150_ACCEL_8G  (1 * _BV(4) | 0 * _BV(3))
#define MPU9150_ACCEL_16G (1 * _BV(4) | 1 * _BV(3))

typedef struct mpu9150_s mpu9150_t;

struct mpu9150_s {
  float mag_adj[3];
};

int8_t mpu9150_init(mpu9150_t *m);

int8_t mpu9150_mag_self_test(mpu9150_t *m, int16_t r[3]);
int8_t mpu9150_mag_single_measurement(mpu9150_t *m, int16_t r[3]);

int8_t mpu9150_gyro_self_test(mpu9150_t *m, float r[3]);
int8_t mpu9150_gyro_single_measurement(mpu9150_t *m, int16_t r[3]);
int8_t mpu9150_gyro_read_offset(mpu9150_t *m, int16_t r[3]);
int8_t mpu9150_gyro_write_offset(mpu9150_t *m, int16_t r[3]);

int8_t mpu9150_accel_self_test(mpu9150_t *m, float r[3]);
int8_t mpu9150_accel_single_measurement(mpu9150_t *m, int16_t r[3]);
int8_t mpu9150_accel_read_offset(mpu9150_t *m, int16_t r[3]);
int8_t mpu9150_accel_write_offset(mpu9150_t *m, int16_t r[3]);

int8_t mpu9150_temp_single_measurement(mpu9150_t *m, float *degc);


// todo

int8_t mpu9150_set_gyro_range(uint8_t range);

int8_t mpu9150_set_accel_range(uint8_t range);

// fifo stuff
#if MPU9150_FIFO
int8_t mpu9150_fifo_init(mpu9150_t *m);
int8_t mpu9150_fifo_read_len(mpu9150_t *m, int16_t *len);
int8_t mpu9150_fifo_read(mpu9150_t *m, mpu9150_fifo_data_t *data);
#endif

#endif
