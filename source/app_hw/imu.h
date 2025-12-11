/**
 * @file imu.h
 * @author Joe Krachey (jkrachey@wisc.edu)
 * @brief 
 * @version 0.1
 * @date 2025-09-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __IMU_H__
#define __IMU_H__

#include <stdint.h>
#include <stdbool.h>

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cyhal_hw_types.h"
#include "spi.h"

#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>


// LSM6DSMTR register addresses
#define IMU_REG_INT1_CTRL      0x0D
#define IMU_REG_CTRL1_XL     0x10  // Accelerometer control
#define IMU_REG_CTRL2_G      0x11  // Gyroscope control
#define IMU_REG_OUTX_L_G     0x22  // Gyro output start
#define IMU_REG_OUTX_L_XL    0x28  // Accel output start
#define IMU_REG_WHO_AM_I     0x0F  // Device ID
#define IMU_REG_CTRL3_C      0x12  // Control register 3

// Configuration values
#define ODR_104HZ    0x40  // Output data rate = 104 Hz
#define FS_XL_2G     0x00  // ±2g
#define FS_G_250DPS  0x00  // ±250 dps
#define IMU_INT1_DRDY_G        0x02    // Gyro data-ready interrupt
#define IMU_CTRL1_XL_208HZ_2G  0x53 // 208 Hz, ±2g
#define IMU_CTRL2_G_208HZ_250DPS 0x50 // 208 Hz, ±250 dps

// Conversion factors
#define ACCEL_SENS_2G   (2.0f / 32768.0f)   // g/LSB
#define GYRO_SENS_250DPS (250.0f / 32768.0f) // dps/LSB

typedef struct {
    float Q_angle;  // Process noise variance for the angle
    float Q_bias;   // Process noise variance for the bias
    float R_measure; // Measurement noise variance

    float angle;    // The angle calculated by the Kalman filter
    float bias;     // The bias calculated by the Kalman filter
    float rate;     // The rate of change of the angle

    float P[2][2];  // Error covariance matrix
} KalmanFilter;

bool imu_init(cyhal_spi_t *spi_obj, cyhal_gpio_t cs_pin);
void imu_write_reg(uint8_t reg, uint8_t value);
uint8_t imu_read_reg(uint8_t reg);
void imu_read_registers(uint8_t reg, uint8_t *buffer, uint8_t length);
void imu_read_accel_gyro(int32_t *ax, int32_t *ay, int32_t *az, int32_t *gx, int32_t *gy, int32_t *gz);
void imu_kalman_init(KalmanFilter *kf);
float imu_kalman_update(KalmanFilter *kf, float newAngle, float newRate, float dt);
float imu_get_pitch(float ax, float ay, float az);
float imu_get_roll(float ax, float ay, float az);
#define M_PI 3.14159265358979323846

#endif