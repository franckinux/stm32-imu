target_include_directories(${MY_TARGET}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/application/inc
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/inc
)

target_sources(${MY_TARGET}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/application/src/init.c
        ${CMAKE_CURRENT_SOURCE_DIR}/application/src/led.c
        ${CMAKE_CURRENT_SOURCE_DIR}/application/src/gps.c
        ${CMAKE_CURRENT_SOURCE_DIR}/application/src/imu.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/src/fifo.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/src/fifos.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/src/i2c_tools.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/src/log.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/src/l3gd20.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/src/lsm303dlhc.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/src/minmea.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/src/mpu6050.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/src/printf.c
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/src/qmc5883l.c
)

target_include_directories(${MY_TARGET} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/cocoos/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/os/inc
)

target_sources(${MY_TARGET} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/cocoos/src/os_assert.c
    ${CMAKE_CURRENT_SOURCE_DIR}/cocoos/src/os_cbk.c
    ${CMAKE_CURRENT_SOURCE_DIR}/cocoos/src/os_event.c
    ${CMAKE_CURRENT_SOURCE_DIR}/cocoos/src/os_kernel.c
    ${CMAKE_CURRENT_SOURCE_DIR}/cocoos/src/os_msgqueue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/cocoos/src/os_sem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/cocoos/src/os_task.c
)

set(STM_HAL_DEVICE "STM32F401xC")
set(STM_USE_HAL_DRIVER true)
set(STM_USE_FULL_LL_DRIVER true)
set(STM_USE_FULL_ASSERT false)

target_compile_definitions(${MY_TARGET} PUBLIC ${STM_HAL_DEVICE})
target_compile_definitions(${MY_TARGET} PUBLIC ${STM_USE_HAL_DRIVER})
target_compile_definitions(${MY_TARGET} PUBLIC ${STM_USE_FULL_LL_DRIVER})
target_compile_definitions(${MY_TARGET} PUBLIC ${STM_USE_FULL_ASSERT})

add_subdirectory(stm32f401)

target_link_options(${MY_TARGET} PRIVATE -T${LINKER_SCRIPT})
target_link_options(${MY_TARGET} PRIVATE -Wl,-Map=${MY_TARGET}.map)
target_link_libraries(${MY_TARGET} stm32f4xx-hal ${EXTERNAL_LIBRARIES})
