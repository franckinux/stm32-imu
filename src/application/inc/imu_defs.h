#ifndef IMU_H
#define IMU_H

// #define MPU6050
// #define QMC5883L
#define LSM303DLHC
#define L3GD20
/* #define TILT_COMPENSATION */

#include <stdint.h>

typedef struct
{
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    int16_t Mag_X_RAW;
    int16_t Mag_Y_RAW;
    int16_t Mag_Z_RAW;
    double Mx;
    double My;
    double Mz;

    float Temperature;
} Imu_t;

// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

extern Imu_t imu;

#endif
