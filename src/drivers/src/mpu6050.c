/*
 * mpu6050.c
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 *
 *  Contact information
 *  -------------------
 *
 * e-mail   :  leech001@gmail.com
 */

/*
 * |---------------------------------------------------------------------------------
 * | Copyright (C) Bulanov Konstantin,2021
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |---------------------------------------------------------------------------------
 */

/* Modified by Bobble  */

#include <stdbool.h>
#include "main.h"
#include "i2c_tools.h"
#include "mpu6050.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

extern I2C_HandleTypeDef hi2c1;
static const uint16_t i2c_timeout = 100;
static uint8_t Rec_Data[14];  // acc: 6, temp: 2, gyro: 6. total = 14

const double Accel_Z_corrector = 14418.0;

HAL_StatusTypeDef MPU6050_Init(void)
{
  uint8_t Data;
  HAL_StatusTypeDef status;

  status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_RA_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
  if (status != HAL_OK) {
    return status;
  }

  // check device ID WHO_AM_I
  if (Data == 104) {  // 0x68 will be returned by the sensor if everything goes well
    // reset the chip
    Data = 1 << MPU6050_PWR1_DEVICE_RESET_BIT;
    status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    if (status != HAL_OK) {
      return status;
    }
    do {
      status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
      if (status != HAL_OK) {
        return status;
      }
      HAL_Delay(2);
    } while (Data & (1 << MPU6050_PWR1_DEVICE_RESET_BIT));
    // power management register 0X6B we should write all 0's to wake the sensor up
    Data = 0;
    status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    if (status != HAL_OK) {
      return status;
    }

    // Set DATA RATE of 32Hz by writing SMPLRT_DIV register
    Data = 0xff;
    status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_RA_SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    if (status != HAL_OK) {
      return status;
    }

    // Set accelerometer configuration in ACCEL_CONFIG Register
    // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> +/- 2g
    Data = 0;
    status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    if (status != HAL_OK) {
      return status;
    }

    // Set Gyroscopic configuration in GYRO_CONFIG Register
    // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> +/- 250 °/s
    Data = 0;
    status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    if (status != HAL_OK) {
      return status;
    }

    return HAL_OK;
  } else {
    return HAL_ERROR;
  }
}


void read_mpu6050_values(void)
{
  HAL_StatusTypeDef status;
  int16_t temp;

  // Start a DMA transfer from the i2c device to the memory
  status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, Rec_Data, sizeof(Rec_Data), i2c_timeout);
  check_i2c_status(&hi2c1, status);

  imu.Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
  imu.Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
  imu.Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
  temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
  imu.Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
  imu.Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
  imu.Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

  imu.Ax = imu.Accel_X_RAW / 16384.0;
  imu.Ay = imu.Accel_Y_RAW / 16384.0;
  imu.Az = imu.Accel_Z_RAW / Accel_Z_corrector;
  imu.Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
  imu.Gx = imu.Gyro_X_RAW / 131.0;
  imu.Gy = imu.Gyro_Y_RAW / 131.0;
  imu.Gz = imu.Gyro_Z_RAW / 131.0;
}
