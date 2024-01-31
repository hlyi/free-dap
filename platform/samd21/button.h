#ifndef _BUTTON_H_
#define _BUTTON_H_

#include "hal_config.h"

//#define BUTTON_DEBUG

// debounce set to 10ms
#define BUTTON_DEBOUNCE_TIME    10
#define BUTTON_CLICK_MIN        40
#define BUTTON_CLICK_MAX        250
#define BUTTON_HOLD_MIN         700
enum button_mode {button_idle, button_click, button_doubletap, button_tripletap, button_hold, button_tap_hold, button_doubletap_hold};

extern uint64_t			app_system_time;

inline static void button_init(void) {
	HAL_GPIO_BUTTON_in();
//	HAL_GPIO_BUTTON_pullup(); Jeff Probe has an external pullup
}

void button_task(void);

#endif
