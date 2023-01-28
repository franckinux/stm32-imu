#include <stdbool.h>
#include "main.h"
#include "i2c_tools.h"
#include "imu_defs.h"
#include "qmc5883l.h"

extern I2C_HandleTypeDef hi2c2;
static const uint16_t i2c_timeout = 100;
static uint8_t Rec_Data[7];  // mag: 6, status: 1. total = 7


HAL_StatusTypeDef qmc5883l_initialize(void)
{
  uint8_t Data;
  HAL_StatusTypeDef status;

  status = HAL_I2C_Mem_Read(&hi2c2, QMC5883L_ADDRESS, QMC5883L_CHIP_ID, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
  if (status != HAL_OK) {
    return status;
  }

  if (Data == 0xff) {
    Data = 1;
    status = HAL_I2C_Mem_Write(&hi2c2, QMC5883L_ADDRESS, QMC5883L_FBR_PERIOD, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    if (status != HAL_OK) {
      return status;
    }
    HAL_Delay(5);

    /* Data = QMC5883L_CONFIG_ROLLING_ADDRESS | QMC5883L_CONFIG_INTERRUPT_ENABLE; */
    // Attention : QMC5883L_CONFIG_INTERRUPT_ENABLE is active at 0 !
    Data = QMC5883L_CONFIG_INTERRUPT_ENABLE ;
    status = HAL_I2C_Mem_Write(&hi2c2, QMC5883L_ADDRESS, QMC5883L_CONTROL_2, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    if (status != HAL_OK) {
      return status;
    }

    Data = QMC5883L_CONFIG_CONT | QMC5883L_CONFIG_10HZ | QMC5883L_CONFIG_2GAUSS | QMC5883L_CONFIG_OS512;
    status = HAL_I2C_Mem_Write(&hi2c2, QMC5883L_ADDRESS, QMC5883L_CONTROL_1, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    if (status != HAL_OK) {
      return status;
    }

    return HAL_OK;
  } else {
    return HAL_ERROR;
  }
}


void read_qmc5883l_values(void)
{
  HAL_StatusTypeDef status;

  // Start a DMA transfer from the i2c device to the memory
  // as data are read before the status register, this clears the DRDY bit
  status = HAL_I2C_Mem_Read(&hi2c2, QMC5883L_ADDRESS, QMC5883L_X_LSB, I2C_MEMADD_SIZE_8BIT, Rec_Data, sizeof(Rec_Data), i2c_timeout);
  check_i2c_status(&hi2c2, status);

  imu.Mag_X_RAW = (int16_t) ((Rec_Data[1] << 8) | Rec_Data[0]);
  imu.Mag_Y_RAW = (int16_t) ((Rec_Data[3] << 8) | Rec_Data[2]);
  imu.Mag_Z_RAW = (int16_t) ((Rec_Data[5] << 8) | Rec_Data[4]);
}
