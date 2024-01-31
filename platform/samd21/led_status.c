#include "hal_config.h"
#include "led_status.h"
#include "pwr_ctrl.h"


static uint64_t		pwr_led_time = 0;
static uint64_t		vcp_led_time = 0;
static uint64_t		app_led_time = 0;

static void pwr_led_task(void)
{
	static uint8_t	flash_cnt = 0;

	bool pwr_led_on = false;
	if ( is_tgt_power_detected() ){
		// power detected
		if ( is_tgt_power_driven() ) {
			 // probe provides power supply, turn on LED
			 pwr_led_on = true;
		}else{
			 // probe doesn't provide power interface
			 // blinking LED
			 pwr_led_on = flash_cnt > 1;
			 flash_cnt++;
			 if ( flash_cnt > 19 ) flash_cnt = 0;
		}
	}
	TCC0->CC[PWR_LED_CC_CH].reg = pwr_led_on ? LED_BRIGHTNESS : 0 ;
}

static void app_led_task(void)
{
	static bool	state = false;
	if ( app_event ) {
		state = !state;
	}else {
		state = true;
	}
	TCC0->CC[APP_LED_CC_CH].reg = state ? LED_BRIGHTNESS : 0 ;
	app_event = false;
}

static void vcp_led_task(void)
{
	static bool	state = false;
	if ( vcp_event ) {
		state = !state;
	}else {
		state = vcp_opened;
	}
	TCC0->CC[VCP_LED_CC_CH].reg = state ? LED_BRIGHTNESS : 0 ;
	vcp_event = false;
}

void led_status_init(void)
{
	HAL_GPIO_PWR_LED_out();
	HAL_GPIO_PWR_LED_clr();
	HAL_GPIO_VCP_LED_out();
	HAL_GPIO_VCP_LED_clr();
	HAL_GPIO_APP_LED_out();
	HAL_GPIO_APP_LED_clr();

	pwr_led_time = PWR_LED_INTERVAL;
	vcp_led_time = VCP_LED_INTERVAL;
	app_led_time = APP_LED_INTERVAL;

	/* Enable the APB clock for TCC0 */
	PM->APBCMASK.reg |= PM_APBCMASK_TCC0;
	/* Enable GCLK1 and wire it up to TCC0 and TCC1. */
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |GCLK_CLKCTRL_ID_TCC0_TCC1 | GCLK_CLKCTRL_GEN(1);
	/* Wait until the clock bus is synchronized. */
	while (GCLK->STATUS.bit.SYNCBUSY) {};

	/* Configure the clock prescaler for each TCC.
		 This lets you divide up the clocks frequency to make the TCC count slower
		 than the clock. In this case, 8MHz clock by 16 making the
		 TCC operate at 500kHz. This means each count (or "tick") is 2us.
	*/
	TCC0->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV16_Val);

	TCC0->PER.reg = PWM_LED_PERIOD;
	while (TCC0->SYNCBUSY.bit.PER) {};

	/* Use "Normal PWM" */
	TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
	/* Wait for bus synchronization */
	while (TCC0->SYNCBUSY.bit.WAVE) {};

	/* n for CC[n] is determined by n = x % 4 where x is from WO[x]
	 WO[x] comes from the peripheral multiplexer - we'll get to that in a second.
	*/
	TCC0->CC[APP_LED_CC_CH].reg = LED_BRIGHTNESS ;
	while (TCC0->SYNCBUSY.bit.CC3) {};

	TCC0->CC[VCP_LED_CC_CH].reg = LED_BRIGHTNESS ;
	while (TCC0->SYNCBUSY.bit.CC2) {};

	TCC0->CC[PWR_LED_CC_CH].reg = LED_BRIGHTNESS ;
	while (TCC0->SYNCBUSY.bit.CC0) {};

	TCC0->CTRLA.reg |= (TCC_CTRLA_ENABLE);
	while (TCC0->SYNCBUSY.bit.ENABLE) {};

	HAL_GPIO_APP_LED_pmuxen( HAL_GPIO_PMUX_F );
	HAL_GPIO_APP_LED_clr();

	/* set alt func */
	HAL_GPIO_VCP_LED_out();
	HAL_GPIO_VCP_LED_clr();
	HAL_GPIO_VCP_LED_pmuxen( HAL_GPIO_PMUX_F );

	/* set alt func */
	HAL_GPIO_PWR_LED_out();
	HAL_GPIO_PWR_LED_clr();
	HAL_GPIO_PWR_LED_pmuxen( HAL_GPIO_PMUX_F );
}


void led_status_task(void)
{
	if ( app_system_time >= pwr_led_time ){
		pwr_led_time = app_system_time + PWR_LED_INTERVAL;
		pwr_led_task();
	}
	if ( app_system_time >= vcp_led_time ){
		vcp_led_time = app_system_time + VCP_LED_INTERVAL;
		vcp_led_task();
	}
	if ( app_system_time >= app_led_time ){
		app_led_time = app_system_time + APP_LED_INTERVAL;
		app_led_task();
	}
}
