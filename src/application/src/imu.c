#include <stdbool.h>
#include <math.h>
#include "main.h"
#include "os_applAPI.h"
/* #include "imu.h" */
#include "imu_defs.h"
#ifdef QMC5883L
#include "qmc5883l.h"
#endif
#ifdef MPU6050
#include "mpu6050.h"
#endif
#ifdef LSM303DLHC
#include "lsm303dlhc.h"
#endif
#ifdef L3GD20
#include "l3gd20.h"
#endif
#include "log.h"
#include "printf.h"

#define LOG_LEVEL LOG_DEBUG
#define LOG_MODULE_NAME "position"
#define CYCLE_PERIOD 50  // 50 ms
#define PRINT_PERIOD 1000  // 1s

#define RAD_TO_DEG 57.29577951308232
#define DEG_TO_RAD 0.017453292519943295

Imu_t imu;

static int counter = 0;

static void init(void)
{
  bool ok = true;
#ifdef QMC5883L
  ok = ok && (qmc5883l_initialize() == HAL_OK);
#endif
#ifdef MPU6050
  ok = ok && (MPU6050_Init() == HAL_OK);
#endif
#ifdef LSM303DLHC
  ok = ok && (lsm303dlhc_init() == HAL_OK);
#endif
#ifdef L3GD20
  ok = ok && (L3GD20_Init() == HAL_OK);
#endif

  if (!ok) {
    Error_Handler();
  }
}


void imu_task(void)
{
  char log_buffer[48];
  double acc_pitch_r, acc_roll_r;  // "accel" angles in radians
  static double pitch_r = 0.0, roll_r = 0.0, yaw_r; // filtered angles in radians
  double pitch_d, roll_d, yaw_d; // filtered angles in degrees
  double cp, sp, cr, sr;   // sin and cos of pich and roll angles
  double gz2, by2, bz2, bx3;  // intermediary values
  double dt; // in seconds

  task_open();

  while(true) {
    static bool do_init = true;

    if (do_init) {
      init();
      do_init = false;
    }

    task_wait(CYCLE_PERIOD);

// initiate a dma transfer on all values
#ifdef MPU6050
    // accelerometer and gyroscope
    read_mpu6050_values();
#endif
#ifdef QMC5883L
    // magnetometer onky
    read_qmc5883l_values();
#endif
#ifdef LSM303DLHC
    // accelerometer and magnetometer
    read_lsm303dlhc_acc_values();
    read_lsm303dlhc_mag_values();
#endif
#ifdef L3GD20
    // gyroscope onky
    read_l3gd20_values();
#endif

    // reference document:
    // dt0058-computing-tilt-measurement-and-tiltcompensated-ecompass-stmicroelectronics.pdf

    // compute pitch and roll from accel angles along with cos and sin of these
    // pitch and roll unit: radian
    acc_roll_r = atan2(imu.Ay, imu.Az);
    sr = sin(acc_roll_r);
    cr = cos(acc_roll_r);
    gz2 = imu.Ay * sr + imu.Az * cr;
    acc_pitch_r = atan(-imu.Ax/ gz2);
    sp = sin(acc_pitch_r);
    cp = cos(acc_pitch_r);

    // compute filtered pitch and roll (the algorithm is "complementary filter")
    static uint32_t old_tick = 0;
    uint32_t tick = HAL_GetTick();
    if (old_tick == 0) {
      dt = CYCLE_PERIOD / 1000;
    } else {
      dt = (double)(tick - old_tick) / 1000;  // convert milliseconds to seconds
    }
    old_tick = tick;
    // Gy and Gz uinit: degree/s
    pitch_r = 0.8 * (pitch_r + dt * imu.Gy * DEG_TO_RAD) + 0.2 * acc_pitch_r;
    roll_r = 0.8 * (roll_r + dt * imu.Gx * DEG_TO_RAD) + 0.2 * acc_roll_r;
    sr = sin(roll_r);
    cr = cos(roll_r);
    sp = sin(pitch_r);
    cp = cos(pitch_r);

    // compute yaw using filtered pitch and roll
    by2 = imu.Az * sr - imu.Ay * cr;
    bz2 = imu.Ay * sr + imu.Az * cr;
    bx3 = imu.Ax * cp + bz2 * sp;
    yaw_r = atan2(by2, bx3);

    // convert all angles to degrees
    pitch_d = pitch_r * RAD_TO_DEG;
    roll_d = roll_r * RAD_TO_DEG;
    yaw_d = yaw_r * RAD_TO_DEG;

#if ENABLE_LOGS
    counter++;
    if (counter >= PRINT_PERIOD / CYCLE_PERIOD) {
      counter = 0;

      /* snprintf(log_buffer, sizeof(log_buffer), "ax=%d, ay=%d, az=%d", imu.Accel_X_RAW, imu.Accel_Y_RAW, imu.Accel_Z_RAW); */
      /* log_debug(log_buffer); */
      /* snprintf(log_buffer, sizeof(log_buffer), "mx=%d, my=%d, mz=%d", imu.Mag_X_RAW, imu.Mag_Y_RAW, imu.Mag_Z_RAW); */
      /* log_debug(log_buffer); */
      /* snprintf(log_buffer, sizeof(log_buffer), "gx=%d, gy=%d, gz=%d", imu.Gyro_X_RAW, imu.Gyro_Y_RAW, imu.Gyro_Z_RAW); */
      /* log_debug(log_buffer); */
      snprintf(
          log_buffer,
          sizeof(log_buffer),
          "pitch = %.2f, roll = %.2f, yaw=%.2f",
          pitch_d, roll_d, yaw_d
      );
      log_debug(log_buffer);
    }
#else
    UNUSED(roll);
    UNUSED(pitch);
    UNUSED(yaw);
#endif
  }

  task_close();
}
