#include <stdbool.h>
#include "main.h"
#include "os_applAPI.h"
#include "log.h"

#define LOG_LEVEL LOG_INFO
#define LOG_MODULE_NAME "led"
#define CYCLE_PERIOD 500  // 500 ms


void led_task(void)
{
  task_open();

  while(true) {
    task_wait(CYCLE_PERIOD);

    HAL_GPIO_TogglePin(green_led_GPIO_Port, green_led_Pin);

    log_debug("led toggled :-)");
  }

  task_close();
}
