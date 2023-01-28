/**
  ******************************************************************************
  * @file    l3gd20.c
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    26-June-2015
  * @brief   This file provides a set of functions needed to manage the L3GD20,
  *          ST MEMS motion sensor, 3-axis digital output gyroscope.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "l3gd20.h"
#include "i2c_tools.h"
#include "imu_defs.h"

// L3GD20
/* #define GYRO_ADDRESS 0xD6 */
// L3G4200D
#define GYRO_ADDRESS 0xD2

extern I2C_HandleTypeDef hi2c1;
static const uint16_t i2c_timeout = 100;
static uint8_t Rec_Data[6];


/**
  * @brief  Set L3GD20 Initialization.
  * @param  L3GD20_InitStruct: pointer to a L3GD20_InitTypeDef structure
  *         that contains the configuration setting for the L3GD20.
  * @retval None
  */
HAL_StatusTypeDef L3GD20_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t Data;

    /* Read WHO I AM register */
    status = HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDRESS, L3GD20_WHO_AM_I_ADDR, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    if (status != HAL_OK) {
        return status;
    }
    if (Data != I_AM_L3G4200D) {
        return HAL_ERROR;
    }

    /* Write value to MEMS CTRL_REG1 register */
    Data = L3GD20_AXES_ENABLE | L3GD20_MODE_ACTIVE | L3GD20_BANDWIDTH_1 | L3GD20_OUTPUT_DATARATE_1;
    status = HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, L3GD20_CTRL_REG1_ADDR, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    if (status != HAL_OK) {
        return status;
    }

    /* Write value to MEMS CTRL_REG4 register */
    // change sensibility factor (see ReadXYZAng function) if you change full scale
    // lsb first by default
    Data = L3GD20_BlockDataUpdate_Single | L3GD20_BLE_LSB | L3GD20_FULLSCALE_250;
    status = HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, L3GD20_CTRL_REG4_ADDR, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    if (status != HAL_OK) {
      return status;
    }

    return HAL_OK;
}


/**
  * @brief  Reboot memory content of L3GD20
  * @param  None
  * @retval None
  */
void L3GD20_RebootCmd(void)
{
    HAL_StatusTypeDef status;
    uint8_t Data;

    /* Read CTRL_REG5 register */
    status = HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDRESS, L3GD20_CTRL_REG5_ADDR, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    check_i2c_status(&hi2c1, status);

    /* Enable or Disable the reboot memory */
    Data |= L3GD20_BOOT_REBOOTMEMORY;

    /* Write value to MEMS CTRL_REG5 register */
    status = HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, L3GD20_CTRL_REG5_ADDR, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    check_i2c_status(&hi2c1, status);
}


/**
  * @brief Set L3GD20 in low-power mode
  * @param
  * @retval  None
  */
void L3GD20_LowPower(uint8_t reg1)
{
    HAL_StatusTypeDef status;

    /* Write value to MEMS CTRL_REG1 register */
    status = HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, L3GD20_CTRL_REG1_ADDR, I2C_MEMADD_SIZE_8BIT, &reg1, 1, i2c_timeout);
    check_i2c_status(&hi2c1, status);
}


/**
  * @brief  Set L3GD20 Interrupt INT1 configuration
  * @param  data: the configuration setting for the L3GD20 Interrupt.
  * @retval None
  */
void L3GD20_INT1InterruptConfig(uint8_t ctrl_cfr, uint8_t ctrl3)
{
    HAL_StatusTypeDef status;
    uint8_t Data;

    /* Read INT1_CFG register */
    status = HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDRESS, L3GD20_INT1_CFG_ADDR, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    check_i2c_status(&hi2c1, status);

    Data &= 0x80;
    Data |= ctrl_cfr;

    /* Write value to MEMS INT1_CFG register */
    status = HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, L3GD20_INT1_CFG_ADDR, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    check_i2c_status(&hi2c1, status);

    /* Read CTRL_REG3 register */
    status = HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDRESS, L3GD20_CTRL_REG3_ADDR, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    check_i2c_status(&hi2c1, status);

    Data &= 0xDF;
    Data |= ctrl3;

    /* Write value to MEMS CTRL_REG3 register */
    status = HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, L3GD20_CTRL_REG3_ADDR, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    check_i2c_status(&hi2c1, status);
}


/**
  * @brief  Enable INT1 or INT2 interrupt
  * @param  data: choice of INT1 or INT2
  *      This parameter can be:
  *        @arg L3GD20_INT1
  *        @arg L3GD20_INT2
  * @retval None
  */
void L3GD20_EnableIT(uint8_t data)
{
    HAL_StatusTypeDef status;
    uint8_t Data;

    /* Read CTRL_REG3 register */
    status = HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDRESS, L3GD20_CTRL_REG3_ADDR, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    check_i2c_status(&hi2c1, status);

    if (data == L3GD20_INT1) {
        Data &= 0x7F;
        Data |= L3GD20_INT1INTERRUPT_ENABLE;
    } else if (data == L3GD20_INT2) {
        Data &= 0xF7;
        Data |= L3GD20_INT2INTERRUPT_ENABLE;
    }

    /* Write value to MEMS CTRL_REG3 register */
    status = HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, L3GD20_CTRL_REG3_ADDR, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    check_i2c_status(&hi2c1, status);
}


/**
  * @brief  Disable  INT1 or INT2 interrupt
  * @param  data: choice of INT1 or INT2
  *      This parameter can be:
  *        @arg L3GD20_INT1
  *        @arg L3GD20_INT2
  * @retval None
  */
void L3GD20_DisableIT(uint8_t data)
{
    HAL_StatusTypeDef status;
    uint8_t Data;

    /* Read CTRL_REG3 register */
    status = HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDRESS, L3GD20_CTRL_REG3_ADDR, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    check_i2c_status(&hi2c1, status);

    if (data == L3GD20_INT1) {
        Data &= 0x7F;
        Data |= L3GD20_INT1INTERRUPT_DISABLE;
    } else if (data == L3GD20_INT2) {
        Data &= 0xF7;
        Data |= L3GD20_INT2INTERRUPT_DISABLE;
    }

    /* Write value to MEMS CTRL_REG3 register */
    status = HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, L3GD20_CTRL_REG3_ADDR, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    check_i2c_status(&hi2c1, status);
}


/**
  * @brief  Set High Pass Filter Modality
  * @param  data: contains the configuration setting for the L3GD20.
  * @retval None
  */
void L3GD20_FilterConfig(uint8_t data)
{
    HAL_StatusTypeDef status;
    uint8_t Data;

    /* Read CTRL_REG2 register */
    status = HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDRESS, L3GD20_CTRL_REG2_ADDR, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    check_i2c_status(&hi2c1, status);

    Data &= 0xC0;

    /* Configure MEMS: mode and cutoff frequency */
    Data |= data;

    /* Write value to MEMS CTRL_REG2 register */
    status = HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, L3GD20_CTRL_REG2_ADDR, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    check_i2c_status(&hi2c1, status);
}


/**
  * @brief  Enable or Disable High Pass Filter
  * @param  data: new state of the High Pass Filter feature.
  *      This parameter can be:
  *         @arg: L3GD20_HIGHPASSFILTER_DISABLE
  *         @arg: L3GD20_HIGHPASSFILTER_ENABLE
  * @retval None
  */
void L3GD20_FilterCmd(uint8_t data)
{
    HAL_StatusTypeDef status;
    uint8_t Data;

    /* Read CTRL_REG5 register */
    status = HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDRESS, L3GD20_CTRL_REG5_ADDR, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    check_i2c_status(&hi2c1, status);

    Data &= 0xEF;

    Data |= data;

    /* Write value to MEMS CTRL_REG5 register */
    status = HAL_I2C_Mem_Write(&hi2c1, GYRO_ADDRESS, L3GD20_CTRL_REG5_ADDR, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    check_i2c_status(&hi2c1, status);
}


/**
  * @brief  Get status for L3GD20 data
  * @param  None
  * @retval Data status in a L3GD20 Data
  */
uint8_t L3GD20_GetDataStatus(void)
{
    HAL_StatusTypeDef status;
    uint8_t Data;

    /* Read STATUS_REG register */
    status = HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDRESS, L3GD20_STATUS_REG_ADDR, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
    check_i2c_status(&hi2c1, status);

    return Data;
}


/**
* @brief  Calculate the L3GD20 angular data.
* @param  pfData: Data out pointer
* @retval None
*/
void read_l3gd20_values(void)
{
    HAL_StatusTypeDef status;
    float sensitivity = L3GD20_SENSITIVITY_250DPS;

    status = HAL_I2C_Mem_Read(&hi2c1, GYRO_ADDRESS, L3GD20_OUT_X_L_ADDR | 0x80, I2C_MEMADD_SIZE_8BIT, Rec_Data, 6, i2c_timeout);
    check_i2c_status(&hi2c1, status);

    // lsb first
    imu.Gyro_X_RAW = (int16_t) (((uint16_t) Rec_Data[1] << 8) + Rec_Data[0]);
    imu.Gyro_Y_RAW = (int16_t) (((uint16_t) Rec_Data[3] << 8) + Rec_Data[2]);
    imu.Gyro_Z_RAW = (int16_t) (((uint16_t) Rec_Data[5] << 8) + Rec_Data[4]);

    imu.Gx = (double) imu.Gyro_X_RAW * sensitivity;
    imu.Gy = (double) imu.Gyro_Y_RAW * sensitivity;
    imu.Gz = (double) imu.Gyro_Z_RAW * sensitivity;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
