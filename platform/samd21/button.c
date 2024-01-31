#include "button.h"
#include "pwr_ctrl.h"
#include "usb_cdc.h"

static enum button_mode button_state = button_idle;

void button_task(void)
{
	enum key_proc_state { key_proc_idle, key_proc_engaged, key_proc_wait_rel};
	static enum key_proc_state key_state;

#ifdef BUTTON_DEBUG
	static alignas(4) uint8_t	message[14]={'B',':',0,0,0,0,0,0,0,0,',',0,'\n',0};
#endif
	// assuming button has pullup
	static bool last_lvl = true;
	static uint32_t last_time = 0;
	static bool glitch_lvl = true;
	static uint32_t glitch_time = 0;
	static bool glitch_start = false;
	static uint8_t	click_cnt = 0;

	bool lvl = HAL_GPIO_BUTTON_read();
	uint32_t cur_time = app_system_time;
	uint32_t delta ;
	bool changed ;

	delta = cur_time - glitch_time;
	changed = lvl != glitch_lvl;
	if ( changed ) {
		glitch_time = cur_time;
		glitch_lvl = lvl;
	}

	if ( glitch_start && (delta < BUTTON_DEBOUNCE_TIME )) return;

	if ( changed && (!glitch_start)) {
		glitch_start = true;
		return;
	}
	// passed glitch filtering
	glitch_start = false;

	changed = lvl != last_lvl;
	delta = cur_time - last_time;

	if ( changed ) {
		//button toggled
		last_time = cur_time;
		last_lvl = lvl;
		if ( key_state == key_proc_idle ) {
			key_state = key_proc_engaged;
			click_cnt = 0;
			return;
		}
		if ( key_state == key_proc_wait_rel) {
			key_state = key_proc_idle;
			click_cnt = 0;
			return;
		}
		// process in progress
		if ( lvl ) {
			// button release
			if ( (delta < BUTTON_CLICK_MIN) || (delta > BUTTON_CLICK_MAX) ) {
				// press is too short or too long, terminate current click counting
				key_state = key_proc_idle;
				// if not too short, register the click
				if ( delta > BUTTON_CLICK_MAX) click_cnt ++;
				switch ( click_cnt ) {
					case 0:
						return;
						break;
					case 1:
						button_state = button_click;
						break;
					case 2:
						button_state = button_doubletap;
						break;
					default:
						button_state = button_tripletap;
						break;
				}
			}else {
				click_cnt ++;
				return;
			}
		}else {
			// button press
			if ( delta < BUTTON_CLICK_MAX ) return;
			switch ( click_cnt ) {
				case 0:
					return;
					break;
				case 1:
					button_state = button_click;
					break;
				case 2:
					button_state = button_doubletap;
					break;
				default:
					button_state = button_tripletap;
					break;
			}
			click_cnt = 0;
		}
	}else if ( key_state == key_proc_engaged ){
		// button not change
		if ( delta <	BUTTON_CLICK_MIN) {
			// wait is too short, ignore
			return;
		}
		if ( lvl ) {
			// button has release long enough
			if (delta > BUTTON_CLICK_MAX ) {
				// button release long enough
				switch ( click_cnt ) {
				case 0:
					break;
				case 1:
					button_state = button_click;
					break;
				break;
				case 2:
					button_state = button_doubletap;
					break;
				default:
					button_state = button_tripletap;
					break;
				}
				click_cnt = 0;
				key_state = key_proc_idle;
			} else {
				// still waiting for press again
				return;
			}
		} else {
			if ( delta > BUTTON_HOLD_MIN ) {
				switch ( click_cnt ) {
					case 0:
						button_state = button_hold;
						break;
					case 1:
						button_state = button_tap_hold;
						break;
					default:
						button_state = button_doubletap_hold;
						break;
				}
				click_cnt = 0;
				key_state = key_proc_wait_rel;
			}else{
				// still waiting
				return;
			}
		}
	}

	// processing button click
	if ( button_state != button_idle ) {
#ifdef BUTTON_DEBUG
		for (int i = 0; i < 2; i++){
			message[3-i] = "0123456789ABCDEF"[button_state& 0xf];
			button_state >>= 4;
		}
		message[4] = '\n';
		usb_cdc_send(message,5);
#endif
		switch (button_state ) {
			case button_click:
				toggle_tgt_power();
				break;
			case button_doubletap_hold:
				NVIC_SystemReset();
				break;
			default:
				break;
		}
		button_state = button_idle;
	}
}

