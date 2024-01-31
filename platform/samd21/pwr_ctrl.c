#include "pwr_ctrl.h"
#include "usb_cdc.h"

uint16_t	power_level = 0;
uint8_t		power_state = 0;
uint32_t	voltage_readout = 0 ;

/* Assume power control pin is on TCC2 WO1 */
static void pwr_ctrl_sw_init(void)
{
	HAL_GPIO_EXT_PWR_set();
	HAL_GPIO_EXT_PWR_out();

	/* Enable the APB clock for TCC2 */
	PM->APBCMASK.reg |= PM_APBCMASK_TCC2;
	/* Enable GCLK1 and wire it up to TCC2 */
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |GCLK_CLKCTRL_ID_TCC2_TC3 | GCLK_CLKCTRL_GEN(1);
	/* Wait until the clock bus is synchronized. */
	while (GCLK->STATUS.bit.SYNCBUSY) {};

	/* Configure the clock prescaler for TCC2. */
	TCC2->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV1_Val);

	TCC2->PER.reg = PWR_PWM_STEP;
	while (TCC2->SYNCBUSY.bit.PER) {};

	/* Use "Normal PWM" */
	TCC2->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
	/* Wait for bus synchronization */
	while (TCC2->SYNCBUSY.bit.WAVE) {};

	/* Invert ouput */
	TCC2->DRVCTRL.reg |= TCC_DRVCTRL_INVEN1;
	/* n for CC[n] is determined by n = x % 4 where x is from WO[x]
	 WO[x] comes from the peripheral multiplexer - we'll get to that in a second.
	*/
	TCC2->CC[1].reg = 0;
	while (TCC2->SYNCBUSY.bit.CC3) {};

	TCC2->CTRLA.reg |= (TCC_CTRLA_ENABLE);
	while (TCC2->SYNCBUSY.bit.ENABLE) {};
}

void toggle_tgt_power()
{
	if (power_state == 2) return;
	set_tgt_power (!power_state);
}

void set_tgt_power(bool on)
{
#ifdef PWR_DEBUG
	static alignas(4) uint8_t	message[12] = {'P','S','=', 0,'L','=',0,0,'\n',0};
	uint16_t dbg = power_state;
	for (int i = 0; i < 1; i++){
		message[3-i] = "0123456789ABCDEF"[dbg & 0xf];
		dbg >>= 4;
	}
	dbg = power_level;
	for (int i = 0; i < 2; i++){
		message[7-i] = "0123456789ABCDEF"[dbg & 0xf];
		dbg >>= 4;
	}
	usb_cdc_send(message,9);
#endif
//	HAL_GPIO_EXT_PWR_toggle();
//	return;
	// if in power up transition, ignore the request
	if ( power_state == 2) return;				// ignore if if in transition state
	if ( power_state && on ) return;			// ignore if request the state is
	if ( !power_state && ! on) return;		//	the same as current state
	if ( ! on ) {
		// turn off power
		HAL_GPIO_EXT_PWR_set();
		HAL_GPIO_EXT_PWR_pmuxdis();
		power_state = 0;
		return;
	}
	power_state = 2;
	TCC2->CC[1].reg = 0 ;
	TCC2->INTFLAG.reg = TCC_INTFLAG_OVF;
	TCC2->CTRLA.reg |= (TCC_CTRLA_ENABLE);
	while (TCC2->SYNCBUSY.bit.ENABLE) {};
	HAL_GPIO_EXT_PWR_pmuxen(HAL_GPIO_PMUX_E);
}

static void pwr_ctrl_sw_task()
{
	if (power_state != 2 ) return;
	if ( !TCC2->INTFLAG.bit.OVF ) return;
	TCC2->INTFLAG.reg = TCC_INTFLAG_OVF;
	power_level ++;
	if ( power_level < PWR_PWM_STEP ) {
		TCC2->CC[1].reg = power_level;
		return;
	}
	// reach full power on
	HAL_GPIO_EXT_PWR_clr();
	HAL_GPIO_EXT_PWR_pmuxdis();
	TCC2->CTRLA.reg &= ~(TCC_CTRLA_ENABLE);
	power_state = 1;
	power_level = 0;
}

static void adc_init(void)
{
	/* Enable the APB clock for the ADC. */
	PM->APBCMASK.reg |= PM_APBCMASK_ADC;

	/* Enable GCLK1 for the ADC */
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(1) | GCLK_CLKCTRL_ID_ADC;

	/* Wait for bus synchronization. */
	while (GCLK->STATUS.bit.SYNCBUSY) {};

	/* read caliberation data */
	uint32_t bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
	uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
	linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;

	/* Wait for bus synchronization. */
	while (ADC->STATUS.bit.SYNCBUSY) {};

	/* Write the calibration data. */
	ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);
	/* Wait for bus synchronization. */
	while (ADC->STATUS.bit.SYNCBUSY) {};

	/* Use the internal VCC reference. This is 1/2 of what's on VCCA.
		 since VCCA is typically 3.3v, this is 1.65v.
	*/
	ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;

	/* Only capture four samples for better accuracy,
	*/
	ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_8;

	/* Set the clock prescaler to 128, which will run the ADC at
		 8 Mhz / 128 = 62.5 kHz.
		 Set the resolution to 16bit.
	*/
	ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV128 | ADC_CTRLB_RESSEL_16BIT;

	/* Configure the input parameters.

	 - GAIN_DIV2 means that the input voltage is halved. This is important
		 because the voltage reference is 1/2 of VCCA. So if you want to
		 measure 0-3.3v, you need to halve the input as well.

	 - MUXNEG_GND means that the ADC should compare the input value to GND.

	 - MUXPOST_PIN0 means that the ADC should read from AIN0, or PA02.
	*/
	ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_DIV2 | ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_PIN0;

	HAL_GPIO_ADC_PWRSENSE_in();
	HAL_GPIO_ADC_PWRSENSE_pmuxen( HAL_GPIO_PMUX_B );
	HAL_GPIO_ADC_VREF_in();
	HAL_GPIO_ADC_VREF_pmuxen( HAL_GPIO_PMUX_B );

	/* Wait for bus synchronization. */
	while (ADC->STATUS.bit.SYNCBUSY) {};

	/* Enable the ADC. */
	ADC->CTRLA.bit.ENABLE = true;
}

static void adc_task(void)
{
#ifdef ADC_DEBUG
	static alignas(4) uint8_t	message[12] = {'V','=', 0,0,0,0,0,0,0,0, '\n',0};
#endif
	enum adc_state { adc_state_idle, adc_state_sync, adc_state_sample };
	static enum adc_state adc_st = adc_state_idle;
	static uint64_t next_sample_time = ADC_SAMPLE_INTERVAL;

	if ( app_system_time < next_sample_time ) return;

	switch (adc_st ){
		case adc_state_idle:
		case adc_state_sync:
			if ( ! ADC->STATUS.bit.SYNCBUSY ) {
				/* Start the ADC using a software trigger. */
				ADC->SWTRIG.bit.START = true;
				adc_st = adc_state_sample;
			}else{
				adc_st = adc_state_sync;
			}
			break;
		case (adc_state_sample ):
			/* check if the result is ready */
			if (ADC->INTFLAG.bit.RESRDY ) {
				/* Clear the flag. */
				ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
				/* Read the value: voltage in uV = result*148	*/
				/* Due to onboard resistor dividor 100/147 */
				/* (4095*8) = 3.3V */
				voltage_readout = ADC->RESULT.reg;
				adc_st = adc_state_idle;
				next_sample_time = app_system_time + ADC_SAMPLE_INTERVAL;
#ifdef ADC_DEBUG
				uint32_t result = voltage_readout;
				for (int i = 0; i < 8; i++){
					message[9-i] = "0123456789ABCDEF"[result & 0xf];
					result >>= 4;
				}
				usb_cdc_send(message,11);
#endif
			}
			break;
		default:
			// should not be here
			adc_st = adc_state_idle;
			next_sample_time = app_system_time + ADC_SAMPLE_INTERVAL;
		 break;
	}
}

void  pwr_ctrl_init(void)
{
	pwr_ctrl_sw_init();
	adc_init();
}

void  pwr_ctrl_task(void)
{
	pwr_ctrl_sw_task();
	adc_task();
}


