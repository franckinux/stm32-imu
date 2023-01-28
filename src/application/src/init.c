#include "main.h"
#include "os_applAPI.h"
#include "gps.h"
#include "imu.h"
#include "led.h"

uint8_t gps_task_id;
uint8_t imu_task_id;
uint8_t led_task_id;

// events
uint8_t gps_event;


void app_init(void)
{
  // events creation
  gps_event = event_create();

  // tasks creation
  imu_task_id = task_create(imu_task, NULL, 10, NULL, 0, 0);
  gps_task_id = task_create(gps_task, NULL, 20, NULL, 0, 0);
  led_task_id = task_create(led_task, NULL, 30, NULL, 0, 0);
}
