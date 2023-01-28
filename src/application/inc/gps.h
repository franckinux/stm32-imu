#ifndef GPS_APP_H_
#define GPS_APP_H_

#include <stdint.h>

// event
extern uint8_t gps_event;

// task id
uint8_t gps_task_id;

// task entruy point
extern void gps_task(void);

#endif
