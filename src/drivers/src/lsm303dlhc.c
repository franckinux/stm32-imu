#include "stm32f4xx.h"
#include "i2c_tools.h"
#include "imu_defs.h"
#include "lsm303dlhc.h"

#define ACC_ADDRESS 0x32
#define MAG_ADDRESS 0x3C

extern I2C_HandleTypeDef hi2c1;
static const uint16_t i2c_timeout = 100;

static uint8_t Rec_Data[6];


HAL_StatusTypeDef lsm303dlhc_init(void)
{
  uint8_t Data;
  HAL_StatusTypeDef status;

  // reboot memory content
  Data = LSM303DLHC_REG_CTRL5_A_BOOT;
  status = HAL_I2C_Mem_Write(&hi2c1, ACC_ADDRESS, LSM303DLHC_REG_CTRL5_A, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
  if (status != HAL_OK) {
    return status;
  }

  /* configure accelerometer */
  /* enable all three axis and set sample rate */
  Data = (LSM303DLHC_CTRL1_A_XEN | LSM303DLHC_CTRL1_A_YEN | LSM303DLHC_CTRL1_A_ZEN | LSM303DLHC_CTRL1_A_50HZ);
  status = HAL_I2C_Mem_Write(&hi2c1, ACC_ADDRESS, LSM303DLHC_REG_CTRL1_A, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
  if (status != HAL_OK) {
    return status;
  }

  /* update on read, LSB @ low address, scale and high-resolution */
  Data = LSM303DLHC_CTRL4_A_BDU | LSM303DLHC_CTRL4_A_BLE | LSM303DLHC_CTRL4_A_SCALE_2G | LSM303DLHC_CTRL4_A_HR;
  status = HAL_I2C_Mem_Write(&hi2c1, ACC_ADDRESS, LSM303DLHC_REG_CTRL4_A, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
  if (status != HAL_OK) {
    return status;
  }

  /* no interrupt generation */
  Data = LSM303DLHC_CTRL3_A_I1_NONE;
  status = HAL_I2C_Mem_Write(&hi2c1, ACC_ADDRESS, LSM303DLHC_REG_CTRL3_A, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
  if (status != HAL_OK) {
    return status;
  }

  /* configure magnetometer */
  /* set sample rate */
  Data = LSM303DLHC_TEMP_SAMPLE_30HZ;
  status = HAL_I2C_Mem_Write(&hi2c1, MAG_ADDRESS, LSM303DLHC_REG_CRA_M, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
  if (status != HAL_OK) {
    return status;
  }

  /* configure z-axis gain */
  Data = LSM303DLHC_GAIN_3;
  status = HAL_I2C_Mem_Write(&hi2c1, MAG_ADDRESS, LSM303DLHC_REG_CRB_M, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
  if (status != HAL_OK) {
    return status;
  }

  /* set continuous mode */
  Data = LSM303DLHC_MAG_MODE_CONTINUOUS;
  status = HAL_I2C_Mem_Write(&hi2c1, MAG_ADDRESS, LSM303DLHC_REG_MR_M, I2C_MEMADD_SIZE_8BIT, &Data, 1, i2c_timeout);
  if (status != HAL_OK) {
    return status;
  }

  return HAL_OK;
}


HAL_StatusTypeDef read_lsm303dlhc_acc_values(void)
{
  HAL_StatusTypeDef status;

  // set bit 7 of register address for auto-incrementing. see 5.1.2 Digital interfaces Linear acceleration digital interface
  status = HAL_I2C_Mem_Read(&hi2c1, ACC_ADDRESS, LSM303DLHC_REG_OUT_X_L_A | 0x80, I2C_MEMADD_SIZE_8BIT, Rec_Data, sizeof(Rec_Data), i2c_timeout);
  check_i2c_status(&hi2c1, status);

  // msb first
  // the value is 12-bits resolution (HR bit set) left justified. divide by 16 instead of
  // shifting value 4 bits left to propagate the sign and let the compiler
  // optimise the code. the values are in mg (FS bits both unset)
  imu.Accel_X_RAW = (int16_t) (((uint16_t) Rec_Data[0] << 8) + Rec_Data[1]) / 16;
  imu.Accel_Y_RAW = (int16_t) (((uint16_t) Rec_Data[2] << 8) + Rec_Data[3]) / 16;
  imu.Accel_Z_RAW = (int16_t) (((uint16_t) Rec_Data[4] << 8) + Rec_Data[5]) / 16;

  imu.Ax = imu.Accel_X_RAW;
  imu.Ay = imu.Accel_Y_RAW;
  imu.Az = imu.Accel_Z_RAW;

  return HAL_OK;
}


HAL_StatusTypeDef read_lsm303dlhc_mag_values(void)
{
  HAL_StatusTypeDef status;

  // auto_increement is automatic. see §5.1.3 LSM303DLHC Magnetic field digital interface
  status = HAL_I2C_Mem_Read(&hi2c1, MAG_ADDRESS, LSM303DLHC_REG_OUT_X_H_M, I2C_MEMADD_SIZE_8BIT, Rec_Data, sizeof(Rec_Data), i2c_timeout);
  check_i2c_status(&hi2c1, status);

  // msb first
  // the register order is X, Z, Y !
  imu.Mag_X_RAW = (int16_t) (((uint16_t) Rec_Data[0] << 8) + Rec_Data[1]);
  imu.Mag_Y_RAW = (int16_t) (((uint16_t) Rec_Data[4] << 8) + Rec_Data[5]);
  imu.Mag_Z_RAW = (int16_t) (((uint16_t) Rec_Data[2] << 8) + Rec_Data[3]);

  /* gain set to LSM303DLHC_GAIN_3 */
  /* sensitivy is 1 */
  imu.Mx = imu.Mag_X_RAW / 670.0;
  imu.My = imu.Mag_Y_RAW / 670.0;
  imu.Mz = imu.Mag_Z_RAW / 600.0;

  return HAL_OK;
}
