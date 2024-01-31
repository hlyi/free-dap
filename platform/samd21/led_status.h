#ifndef _LED_STATUS_H_
#define _LED_STATUS_H_

#include <stdbool.h>
#include <stdint.h>

#define PWR_LED_INTERVAL	100	//ms 
#define VCP_LED_INTERVAL	250	//ms 
#define APP_LED_INTERVAL	250	//ms 

// based upon 3MHz for TCC
#define PWM_LED_PERIOD		2000
// percentage of LED brigtness, 100 = Full brightness, current set to 5%
#define LED_BRIGHTNESS		(5 * PWM_LED_PERIOD/100)

extern uint64_t		app_system_time;
extern bool		vcp_event;
extern bool		app_event;
extern bool		vcp_opened;

void 			led_status_init(void);
void 			led_status_task(void);


#endif
