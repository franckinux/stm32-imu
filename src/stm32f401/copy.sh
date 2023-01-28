#!/bin/env bash
# Prior to execute this script, open the project file (*.ioc) in ../stm32cubemx
# with the STM3CubeMX software, make the changes and generate the code
# and don't forget to save your changes in theses files !
set -e -x
cp ../../stm32cubemx/STM32F401CCUx_FLASH.ld .
cp ../../stm32cubemx/startup_stm32f401xc.s startup/src/
cp ../../stm32cubemx/Core/Src/main.c startup/src/
cp ../../stm32cubemx/Core/Src/stm32f4xx_hal_msp.c startup/src
cp ../../stm32cubemx/Core/Src/system_stm32f4xx.c startup/src/
cp ../../stm32cubemx/Core/Src/stm32f4xx_it.c startup/src/
cp ../../stm32cubemx/Core/Inc/main.h startup/inc/
cp ../../stm32cubemx/Core/Inc/stm32f4xx_hal_conf.h startup/inc/
cp ../../stm32cubemx/Core/Inc/stm32f4xx_it.h startup/inc/
cp ../../stm32cubemx/Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h startup/inc/
