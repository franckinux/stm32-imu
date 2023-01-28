#include <stdbool.h>
#include "main.h"
#include "os_applAPI.h"
/* #include "gps.h" */
#include "log.h"
#include "minmea.h"
#include "printf.h"

#define LOG_LEVEL LOG_DEBUG
#define LOG_MODULE_NAME "position"
#define CYCLE_PERIOD 50  // 50 ms
#define PRINT_PERIOD 1000  // 1s

typedef enum state {
  NMEA_WAIT_FOR_START = 0,
  NMEA_RECEIVING_FRAME = 1,
  NMEA_RECEIVED_FRAME = 2
} nmea_state_t;

static char nmea_buffer[MINMEA_MAX_SENTENCE_LENGTH + 1];
volatile nmea_state_t nmea_state = NMEA_WAIT_FOR_START;

extern UART_HandleTypeDef huart2;

/* struct minmea_sentence_rmc data; */

void gps_task(void)
{
  char log_buffer[48];

  task_open();

  while(true) {
    struct minmea_sentence_rmc rmc_data;
    double longitude;
    double latitude;
    UNUSED(latitude);
    UNUSED(longitude);

    if (nmea_state == NMEA_RECEIVED_FRAME) {
      if (minmea_sentence_id(nmea_buffer, false) == MINMEA_SENTENCE_RMC) {
        if (minmea_parse_rmc(&rmc_data, nmea_buffer)) {
          if (rmc_data.valid) {
            longitude = minmea_rescale(&rmc_data.longitude, 1000);
            latitude = minmea_rescale(&rmc_data.latitude, 1000);
          }
        }
      }
      nmea_state = NMEA_WAIT_FOR_START;
    }

#if ENABLE_LOGS
    snprintf(log_buffer, sizeof(log_buffer), "latitude = %.4f, longitude=%.4f", latitude, longitude);
    log_debug(log_buffer);
#else
    UNUSED(latitude);
    UNUSED(longitude);
#endif
  }

  task_close();
}

// receiving NMEA frmaes
void USART2_IRQHandler(void)
{
  if ((huart2.Instance->SR & USART_SR_RXNE) != RESET) {
    static int index;

    char c = huart2.Instance->DR;

    switch(nmea_state) {
      case NMEA_WAIT_FOR_START:
        if (c == '$') {
          index = 1;
          nmea_buffer[0] = '$';
          nmea_state = NMEA_RECEIVING_FRAME;
        } else {
          return;
        }
        break;
      case NMEA_RECEIVING_FRAME:
        if (index >= MINMEA_MAX_SENTENCE_LENGTH) {
          nmea_state = NMEA_WAIT_FOR_START;
          return;
        };
        if (c == '\n') {
          nmea_buffer[index++] = '\n';
          if (index >= MINMEA_MAX_SENTENCE_LENGTH) {
            nmea_state = NMEA_WAIT_FOR_START;
            return;
          }
          nmea_buffer[index] = '\0';

          nmea_state = NMEA_RECEIVED_FRAME;
          return;
        } else {
          nmea_buffer[index++] = c;
        }
        break;
      case NMEA_RECEIVED_FRAME:;
    }
  }
}
