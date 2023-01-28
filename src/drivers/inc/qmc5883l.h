#ifndef _qmc5883l_H_
#define _qmc5883l_H_

#include <stdbool.h>
#include "stm32f4xx.h"

#define QMC5883L_ADDRESS            0x1A

/* Register offset */
#define QMC5883L_X_LSB 0
#define QMC5883L_X_MSB 1
#define QMC5883L_Y_LSB 2
#define QMC5883L_Y_MSB 3
#define QMC5883L_Z_LSB 4
#define QMC5883L_Z_MSB 5
#define QMC5883L_STATUS 6
#define QMC5883L_TEMP_LSB 7
#define QMC5883L_TEMP_MSB 8
#define QMC5883L_CONTROL_1 9
#define QMC5883L_CONTROL_2 10
#define QMC5883L_FBR_PERIOD 11
#define QMC5883L_CHIP_ID 13

/* Bit values for the STATUS register */
#define QMC5883L_STATUS_DRDY 1
#define QMC5883L_STATUS_OVL 2
#define QMC5883L_STATUS_DOR 4

/* Oversampling values for the CONFIG register */
#define QMC5883L_CONFIG_OS512 0x00
#define QMC5883L_CONFIG_OS256 0x40
#define QMC5883L_CONFIG_OS128 0x80
#define QMC5883L_CONFIG_OS64  0xc0

/* Range values for the CONFIG register */
#define QMC5883L_CONFIG_2GAUSS 0x00
#define QMC5883L_CONFIG_8GAUSS 0x10

/* Rate values for the CONFIG register */
#define QMC5883L_CONFIG_10HZ 0x00
#define QMC5883L_CONFIG_50HZ 0x04
#define QMC5883L_CONFIG_100HZ 0x08
#define QMC5883L_CONFIG_200HZ 0x0c

/* Mode values for the CONFIG register */
#define QMC5883L_CONFIG_STANDBY 0x00
#define QMC5883L_CONFIG_CONT 0x01

/* Bit values for the CONTROL 2 register */
#define QMC5883L_CONFIG_SOFT_RESET 0x80
#define QMC5883L_CONFIG_ROLLING_ADDRESS 0x40
#define QMC5883L_CONFIG_INTERRUPT_ENABLE 0x01

extern double hdg;

HAL_StatusTypeDef qmc5883l_initialize(void);
void prime_qmc5883l_value(void);
void read_qmc5883l_values(void);

#endif /* _qmc5883l_H_ */
