#ifndef _PWR_CTRL_H
#define _PWR_CTRL_H

#include <stdbool.h>

#include "hal_config.h"

//#define PWR_DEBUG
//#define ADC_DEBUG
//#define PWR_CTRL_INV

extern uint64_t			app_system_time;

#define ADC_SAMPLE_INTERVAL	250				// ms
#define POWER_DETECT_THRESHOLD	7432				// 1.1V

#define PWR_PWM_STEP            64U
extern uint8_t			power_state;			// 0 power off, 1 power on, 2 turning on

extern uint32_t			voltage_readout ;

inline static bool		is_tgt_power_detected(void) { return voltage_readout > POWER_DETECT_THRESHOLD; };
inline static bool		is_tgt_power_driven(void) {
#ifdef PWR_CTRL_INV
					return !HAL_GPIO_EXT_PWR_read() ;
#else
					return HAL_GPIO_EXT_PWR_read() ;
#endif
				};
void     	                set_tgt_power(bool on);
void				toggle_tgt_power(void);

void				pwr_ctrl_init(void);
void				pwr_ctrl_task(void);

#endif

