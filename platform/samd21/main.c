// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024, Alex Taradov <alex@taradov.com>. All rights reserved.

/*- Includes ----------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdalign.h>
#include <string.h>
#include "samd21.h"
#include "hal_config.h"
#include "nvm_data.h"
#include "usb.h"
#include "usb_std.h"
#include "uart.h"
#include "dap.h"
#include "dap_config.h"

/*- Definitions -------------------------------------------------------------*/
#define USB_BUFFER_SIZE		64
#define UART_WAIT_TIMEOUT	10	// ms
#define STATUS_TIMEOUT		250	// ms
#define ADC_SAMPLE_INTERVAL	100	// ms
#define POWER_DETECT_THRESHOLD	7432	// 1.1V

/*- Variables ---------------------------------------------------------------*/
static alignas(4) uint8_t app_req_buf_hid[DAP_CONFIG_PACKET_SIZE];
static alignas(4) uint8_t app_req_buf_bulk[DAP_CONFIG_PACKET_SIZE];
static alignas(4) uint8_t app_req_buf[DAP_CONFIG_PACKET_SIZE];
static alignas(4) uint8_t app_resp_buf[DAP_CONFIG_PACKET_SIZE];
static int app_req_buf_hid_size = 0;
static int app_req_buf_bulk_size = 0;
static bool app_resp_free = true;
static uint64_t app_system_time = 0;
static uint64_t app_status_timeout = 0;
static bool app_dap_event = false;

#ifdef HAL_CONFIG_ENABLE_VCP
static alignas(4) uint8_t app_recv_buffer[USB_BUFFER_SIZE];
static alignas(4) uint8_t app_send_buffer[USB_BUFFER_SIZE];
static int app_recv_buffer_size = 0;
static int app_recv_buffer_ptr = 0;
static int app_send_buffer_ptr = 0;
static bool app_send_buffer_free = true;
static bool app_send_zlp = false;
static uint64_t app_uart_timeout = 0;
static uint64_t app_break_timeout = 0;
static bool app_vcp_event = false;
static bool app_vcp_open = false;
#endif

#ifdef HAL_CONFIG_ENABLE_LED_PWMMODE

// based upon 3MHz for TCC
#define PWM_LED_PERIOD    2000
// percentage of LED brigtness, 100 = Full brightness, current set to 5%
#define LED_BRIGHTNESS    (5 * PWM_LED_PERIOD/100)


extern void custom_hal_gpio_dap_status_toggle();
extern void custom_hal_gpio_dap_status_set();
static void led_custom_init();
#ifdef HAL_CONFIG_ENABLE_VCP
extern void custom_hal_gpio_vcp_status_toggle();
extern void custom_hal_gpio_vcp_status_write(int val);
#endif
#endif

#ifdef HAL_CONFIG_ENABLE_BUTTON
// debounce set to 10ms
#define BUTTON_DEBOUNCE_TIME    10
#define BUTTON_CLICK_MIN        40
#define BUTTON_CLICK_MAX        250
#define BUTTON_HOLD_MIN         700
enum button_mode {button_idle, button_click, button_doubletap, button_tripletap, button_hold, button_tap_hold, button_doubletap_hold};
static enum button_mode button_state = button_idle;
#endif

#ifdef HAL_CONFIG_ENABLE_PROVIDE_PWR
#define PWR_PWM_STEP            64U
volatile uint8_t                power_state = 0;        // 0 power off, 1 power on, 2 turning on
uint16_t                        power_level = 0;
static void                     set_ext_power(bool on);
#endif
//#define BUTTON_DEBUG
//#define ADC_DEBUG
//#define PWR_DEBUG
//#define USBVEN_DEBUG

#ifdef HAL_CONFIG_ENABLE_USB_VEN
#define USB_CMD_PWR_STATUS      0x01
#define USB_CMD_PWR_OFF         0x02
#define USB_CMD_PWR_ON          0x03
#define USB_CMD_BOOT_BL         0x62
#define UF2_BOOTLOADER_MAGIC    0xf01669ef
static bool   reboot_device = false;
#endif

#ifdef HAL_CONFIG_ADC_PWRSENSE
uint32_t     voltage_readout = 0;
#endif
/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
#ifdef HAL_CONFIG_ENABLE_USB_VEN
static bool usb_ven_setup_callback( uint8_t *data, int len)
{
  static alignas(4) uint8_t usb_ret_data[4];
  (void) len;
  bool send_zlp = true;
  if ( data[0] & 0x80 ) {
    // in request
    if ( data[1] == USB_CMD_PWR_STATUS ){
      uint32_t  data = voltage_readout & 0xffffff;
      data |= (power_state) <<24;
      *(uint32_t *)usb_ret_data = data;
      usb_control_send(usb_ret_data, 4);
      send_zlp = false;
    }
  }else {
    // out request
    switch ( data[1] ){
    case USB_CMD_PWR_OFF:
      set_ext_power(false);
      break;
    case USB_CMD_PWR_ON:
      set_ext_power(true);
      break;
    case USB_CMD_BOOT_BL:
      reboot_device = true;
      break;
    }
  }
  if ( send_zlp ) usb_control_send_zlp();
#ifdef USBVEN_DEBUG
  static alignas(4) uint8_t  message[18] = {'U','=', 0,0,',',0,0,',',0,0,0,0,0,0,0,0,'\n',0};
  uint32_t dbg = data[0];
  for (int i = 0; i < 2; i++){
    message[3-i] = "0123456789ABCDEF"[dbg & 0xf];
    dbg >>= 4;
  }
  dbg = data[1];
  for (int i = 0; i < 2; i++){
    message[6-i] = "0123456789ABCDEF"[dbg & 0xf];
    dbg >>= 4;
  }
  dbg = *(uint32_t*)  usb_ret_data;
  for (int i = 0; i < 8; i++){
    message[15-i] = "0123456789ABCDEF"[dbg & 0xf];
    dbg >>= 4;
  }
  usb_cdc_send(message,17);
#endif
  return true;
}
#endif

#ifdef HAL_CONFIG_ENABLE_PROVIDE_PWR
static void pwr_custom_init()
{
  /* Enable the APB clock for TCC2 */
  PM->APBCMASK.reg |= PM_APBCMASK_TCC2;
  /* Enable GCLK1 and wire it up to TCC2 */
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |GCLK_CLKCTRL_ID_TCC2_TC3 |
                    GCLK_CLKCTRL_GEN(1);
  /* Wait until the clock bus is synchronized. */
  while (GCLK->STATUS.bit.SYNCBUSY) {};

  /* Configure the clock prescaler for TCC2.
  */
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

static void ext_power_toggle()
{
  if (power_state == 2) return;
  set_ext_power (!power_state);
}

static void set_ext_power(bool on)
{
#ifdef PWR_DEBUG
  static alignas(4) uint8_t  message[12] = {'P','S','=', 0,'L','=',0,0,'\n',0};
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
//  HAL_GPIO_EXT_PWR_toggle();
//  return;
  // if in power up transition, ignore the request
  if ( power_state == 2) return;        // ignore if if in transition state
  if ( power_state && on ) return;      // ignore if request the state is
  if ( !power_state && ! on) return;    //  the same as current state
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

static void ext_power_task()
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

#endif


#ifdef HAL_CONFIG_ENABLE_BUTTON
static void button_task(void)
{
  enum key_proc_state { key_proc_idle, key_proc_engaged, key_proc_wait_rel};
  static enum key_proc_state key_state;

#ifdef BUTTON_DEBUG
  static alignas(4) uint8_t  message[14]={'B',':',0,0,0,0,0,0,0,0,',',0,'\n',0};
#endif
  // assuming button has pullup
  static bool last_lvl = true;
  static uint32_t last_time = 0;
  static bool glitch_lvl = true;
  static uint32_t glitch_time = 0;
  static bool glitch_start = false;
  static uint8_t  click_cnt = 0;

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
    if ( delta <  BUTTON_CLICK_MIN) {
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
        ext_power_toggle();
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
#endif

#ifdef HAL_CONFIG_ADC_PWRSENSE
static void adc_init(void)
{
  /* Enable the APB clock for the ADC. */
  PM->APBCMASK.reg |= PM_APBCMASK_ADC;

  /* Enable GCLK1 for the ADC */
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
                    GCLK_CLKCTRL_GEN(1) |
                    GCLK_CLKCTRL_ID_ADC;

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
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV128 |
                 ADC_CTRLB_RESSEL_16BIT;

  /* Configure the input parameters.

   - GAIN_DIV2 means that the input voltage is halved. This is important
     because the voltage reference is 1/2 of VCCA. So if you want to
     measure 0-3.3v, you need to halve the input as well.

   - MUXNEG_GND means that the ADC should compare the input value to GND.

   - MUXPOST_PIN0 means that the ADC should read from AIN0, or PA02.
  */
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_DIV2 |
                     ADC_INPUTCTRL_MUXNEG_GND |
                     ADC_INPUTCTRL_MUXPOS_PIN0;

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
  static alignas(4) uint8_t  message[12] = {'V','=', 0,0,0,0,0,0,0,0, '\n',0};
#endif
  enum adc_state { adc_state_idle, adc_state_sync, adc_state_sample };
  static enum adc_state adc_st = adc_state_idle;
  static uint64_t next_sample_time = ADC_SAMPLE_INTERVAL;
  static uint8_t flash_cnt = 0;

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
        /* Read the value: voltage in uV = result*148  */
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
        bool pwr_led_on = false;
        if ( voltage_readout > POWER_DETECT_THRESHOLD ){
          // power detected
          if ( HAL_GPIO_EXT_PWR_read() ) {
             // probe doesn't provide power interface
             // blinking LED
             pwr_led_on = flash_cnt > 1;
             flash_cnt++;
             if ( flash_cnt > 19 ) flash_cnt = 0;
          }else{
             // probe provides power supply, turn on LED
             pwr_led_on = true;
          }
        }
#ifdef HAL_CONFIG_ENABLE_PWR_LED
        TCC0->CC[POWER_LED_CC_CH].reg = pwr_led_on ? LED_BRIGHTNESS : 0 ;
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

#endif


#ifdef HAL_CONFIG_ENABLE_LED_PWMMODE
static void led_custom_init()
{
  /* Enable the APB clock for TCC0 */
  PM->APBCMASK.reg |= PM_APBCMASK_TCC0;
  /* Enable GCLK1 and wire it up to TCC0 and TCC1. */
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |GCLK_CLKCTRL_ID_TCC0_TCC1 |
                    GCLK_CLKCTRL_GEN(1);
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
  TCC0->CC[DAP_STATUS_CC_CH].reg = LED_BRIGHTNESS ;
  while (TCC0->SYNCBUSY.bit.CC3) {};

#ifdef HAL_CONFIG_ENABLE_VCP
  TCC0->CC[VCP_STATUS_CC_CH].reg = LED_BRIGHTNESS ;
  while (TCC0->SYNCBUSY.bit.CC2) {};
#endif

#ifdef HAL_CONFIG_ENABLE_PWR_LED
  TCC0->CC[POWER_LED_CC_CH].reg = LED_BRIGHTNESS ;
  while (TCC0->SYNCBUSY.bit.CC0) {};
#endif

  TCC0->CTRLA.reg |= (TCC_CTRLA_ENABLE);
  while (TCC0->SYNCBUSY.bit.ENABLE) {};

  HAL_GPIO_DAP_STATUS_pmuxen( HAL_GPIO_PMUX_F );
  HAL_GPIO_DAP_STATUS_clr();

#ifdef HAL_CONFIG_ENABLE_VCP
  /* set alt func */
  HAL_GPIO_VCP_STATUS_out();
  HAL_GPIO_VCP_STATUS_clr();
  HAL_GPIO_VCP_STATUS_pmuxen( HAL_GPIO_PMUX_F );
#endif

#ifdef HAL_CONFIG_ENABLE_PWR_LED
  /* set alt func */
  HAL_GPIO_PWR_STATUS_out();
  HAL_GPIO_PWR_STATUS_clr();
  HAL_GPIO_PWR_STATUS_pmuxen( HAL_GPIO_PMUX_F );
#endif

}

void custom_hal_gpio_dap_status_toggle()
{
  static int last_value = 0;
  last_value = ! last_value;
  TCC0->CC[DAP_STATUS_CC_CH].reg = last_value ? LED_BRIGHTNESS : 0 ;
}
void custom_hal_gpio_dap_status_set()
{
  TCC0->CC[DAP_STATUS_CC_CH].reg = LED_BRIGHTNESS ;
}

#ifdef HAL_CONFIG_ENABLE_VCP
void custom_hal_gpio_vcp_status_toggle()
{
  static int last_value = 0;
  last_value = ! last_value;
  custom_hal_gpio_vcp_status_write(last_value);
}

void custom_hal_gpio_vcp_status_write(int val)
{
  TCC0->CC[VCP_STATUS_CC_CH].reg = val ? LED_BRIGHTNESS : 0 ;
}

#endif

#ifdef HAL_CONFIG_ENABLE_PWR_LED
void custom_hal_gpio_pwr_status_set()
{
  TCC0->CC[POWER_LED_CC_CH].reg = LED_BRIGHTNESS ;
}
#endif

#endif

static void custom_init(void)
{
#ifdef HAL_CONFIG_ENABLE_LED_PWMMODE
  led_custom_init();
#endif

#ifdef HAL_CONFIG_ENABLE_PWR_LED
  HAL_GPIO_PWR_STATUS_out();
#ifdef HAL_CONFIG_ENABLE_LED_PWMMODE
  custom_hal_gpio_pwr_status_set();
#else
  HAL_GPIO_PWR_STATUS_set();
#endif
#endif

#ifdef HAL_CONFIG_ADC_PWRSENSE
  adc_init();
#endif

#ifdef HAL_CONFIG_ENABLE_PROVIDE_PWR
  HAL_GPIO_EXT_PWR_out();
  HAL_GPIO_EXT_PWR_set();
  pwr_custom_init();
#endif
#ifdef HAL_CONFIG_ENABLE_USB_VEN
  usb_setup_recv(usb_ven_setup_callback);
#endif
}

static void sys_init(void)
{
  uint32_t coarse, fine;

  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_RWS(1);

  SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33RDY | SYSCTRL_INTFLAG_BOD33DET |
      SYSCTRL_INTFLAG_DFLLRDY;

  coarse = NVM_READ_CAL(NVM_DFLL48M_COARSE_CAL);
  fine = NVM_READ_CAL(NVM_DFLL48M_FINE_CAL);

  SYSCTRL->DFLLCTRL.reg = 0; // See Errata 9905
  while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));

  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_MUL(48000);
  SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE(coarse) | SYSCTRL_DFLLVAL_FINE(fine);

  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_USBCRM |
      SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_CCDIS;

  while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC(GCLK_SOURCE_DFLL48M) |
      GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

#if (defined (HAL_CONFIG_ENABLE_LED_PWMMODE) || defined (HAL_CONFIG_ADC_PWRSENSE))
  // enable GCLK1 for peripherals, base clock is 8MHz same as OSC8M but has
  // better accuracy when USB connected
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(1) | GCLK_GENCTRL_SRC(GCLK_SOURCE_DFLL48M) |
      GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(1) | GCLK_GENDIV_DIV(6);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  custom_init();
#endif

}

//-----------------------------------------------------------------------------
static void serial_number_init(void)
{
  uint32_t wuid[4];
  uint8_t *uid = (uint8_t *)wuid;
  uint32_t sn = 5381;

  wuid[0] = *(volatile uint32_t *)0x0080a00c;
  wuid[1] = *(volatile uint32_t *)0x0080a040;
  wuid[2] = *(volatile uint32_t *)0x0080a044;
  wuid[3] = *(volatile uint32_t *)0x0080a048;

  for (int i = 0; i < 16; i++)
    sn = ((sn << 5) + sn) ^ uid[i];

  for (int i = 0; i < 8; i++)
    usb_serial_number[i] = "0123456789ABCDEF"[(sn >> (i * 4)) & 0xf];

  usb_serial_number[8] = 0;
}

//-----------------------------------------------------------------------------
static void sys_time_init(void)
{
  SysTick->VAL  = 0;
  SysTick->LOAD = F_CPU / 1000ul;
  SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
  app_system_time = 0;
}

//-----------------------------------------------------------------------------
static void sys_time_task(void)
{
  if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
    app_system_time++;
}

#ifdef HAL_CONFIG_ENABLE_VCP
//-----------------------------------------------------------------------------
static void tx_task(void)
{
  while (app_recv_buffer_size)
  {
    if (!uart_write_byte(app_recv_buffer[app_recv_buffer_ptr]))
      break;

    app_recv_buffer_ptr++;
    app_recv_buffer_size--;
    app_vcp_event = true;

    if (0 == app_recv_buffer_size)
      usb_cdc_recv(app_recv_buffer, sizeof(app_recv_buffer));
  }
}

//-----------------------------------------------------------------------------
static void send_buffer(void)
{
  app_send_buffer_free = false;
  app_send_zlp = (USB_BUFFER_SIZE == app_send_buffer_ptr);

  usb_cdc_send(app_send_buffer, app_send_buffer_ptr);

  app_send_buffer_ptr = 0;
}

//-----------------------------------------------------------------------------
static void rx_task(void)
{
  int byte;

  if (!app_send_buffer_free)
    return;

  while (uart_read_byte(&byte))
  {
    int state = (byte >> 8) & 0xff;

    app_uart_timeout = app_system_time + UART_WAIT_TIMEOUT;
    app_vcp_event = true;

    if (state)
    {
      usb_cdc_set_state(state);
    }
    else
    {
      app_send_buffer[app_send_buffer_ptr++] = byte;

      if (USB_BUFFER_SIZE == app_send_buffer_ptr)
      {
        send_buffer();
        break;
      }
    }
  }
}

//-----------------------------------------------------------------------------
static void break_task(void)
{
  if (app_break_timeout && app_system_time > app_break_timeout)
  {
    uart_set_break(false);
    app_break_timeout = 0;
  }
}

//-----------------------------------------------------------------------------
static void uart_timer_task(void)
{
  if (app_uart_timeout && app_system_time > app_uart_timeout)
  {
    if (app_send_zlp || app_send_buffer_ptr)
      send_buffer();

    app_uart_timeout = 0;
  }
}

//-----------------------------------------------------------------------------
void usb_cdc_line_coding_updated(usb_cdc_line_coding_t *line_coding)
{
  uart_init(line_coding);
}

//-----------------------------------------------------------------------------
void usb_cdc_control_line_state_update(int line_state)
{
  bool status = line_state & USB_CDC_CTRL_SIGNAL_DTE_PRESENT;

  app_vcp_open        = status;
  app_send_buffer_ptr = 0;
  app_uart_timeout    = 0;
  app_break_timeout   = 0;

  if (app_vcp_open)
    uart_init(usb_cdc_get_line_coding());
  else
    uart_close();
}

//-----------------------------------------------------------------------------
void usb_cdc_send_break(int duration)
{
  if (USB_CDC_BREAK_DURATION_DISABLE == duration)
  {
    app_break_timeout = 0;
    uart_set_break(false);
  }
  else if (USB_CDC_BREAK_DURATION_INFINITE == duration)
  {
    app_break_timeout = 0;
    uart_set_break(true);
  }
  else
  {
    app_break_timeout = app_system_time + duration;
    uart_set_break(true);
  }
}

//-----------------------------------------------------------------------------
void usb_cdc_send_callback(void)
{
  app_send_buffer_free = true;
}

//-----------------------------------------------------------------------------
void usb_cdc_recv_callback(int size)
{
  app_recv_buffer_ptr = 0;
  app_recv_buffer_size = size;
}
#endif // HAL_CONFIG_ENABLE_VCP

//-----------------------------------------------------------------------------
void usb_hid_send_callback(void)
{
  app_resp_free = true;
}

//-----------------------------------------------------------------------------
void usb_hid_recv_callback(int size)
{
  app_req_buf_hid_size = size;
}

//-----------------------------------------------------------------------------
static void usb_bulk_send_callback(void)
{
  app_resp_free = true;
}

//-----------------------------------------------------------------------------
static void usb_bulk_recv_callback(int size)
{
  app_req_buf_bulk_size = size;
}

//-----------------------------------------------------------------------------
static void dap_task(void)
{
  int interface, size;

  if (!app_resp_free)
    return;

  if (app_req_buf_hid_size)
  {
    interface = USB_INTF_HID;
    size = app_req_buf_hid_size;
    app_req_buf_hid_size = 0;

    memcpy(app_req_buf, app_req_buf_hid, size);

    usb_hid_recv(app_req_buf_hid, sizeof(app_req_buf_hid));
  }
  else if (app_req_buf_bulk_size)
  {
    interface = USB_INTF_BULK;
    size = app_req_buf_bulk_size;
    app_req_buf_bulk_size = 0;

    memcpy(app_req_buf, app_req_buf_bulk, size);

    usb_recv(USB_BULK_EP_RECV, app_req_buf_bulk, sizeof(app_req_buf_bulk));
  }
  else
  {
    return;
  }

  size = dap_process_request(app_req_buf, size, app_resp_buf, sizeof(app_resp_buf));

  if (USB_INTF_BULK == interface)
    usb_send(USB_BULK_EP_SEND, app_resp_buf, size);
  else
    usb_hid_send(app_resp_buf, sizeof(app_resp_buf));

  app_resp_free = false;
  app_dap_event = true;
}

//-----------------------------------------------------------------------------
void usb_configuration_callback(int config)
{
  app_resp_free = true;
  app_req_buf_hid_size = 0;
  app_req_buf_bulk_size = 0;

  usb_set_send_callback(USB_BULK_EP_SEND, usb_bulk_send_callback);
  usb_set_recv_callback(USB_BULK_EP_RECV, usb_bulk_recv_callback);

  usb_hid_recv(app_req_buf_hid, sizeof(app_req_buf_hid));
  usb_recv(USB_BULK_EP_RECV, app_req_buf_bulk, sizeof(app_req_buf_bulk));

#ifdef HAL_CONFIG_ENABLE_VCP
  usb_cdc_recv(app_recv_buffer, sizeof(app_recv_buffer));

  app_send_buffer_free = true;
  app_send_buffer_ptr = 0;
#endif

  (void)config;
}

//-----------------------------------------------------------------------------
static void status_timer_task(void)
{
  if (app_system_time < app_status_timeout)
    return;

  app_status_timeout = app_system_time + STATUS_TIMEOUT;

  if (app_dap_event)
#ifdef HAL_CONFIG_ENABLE_LED_PWMMODE
    custom_hal_gpio_dap_status_toggle();
#else
    HAL_GPIO_DAP_STATUS_toggle();
#endif
  else
#ifdef HAL_CONFIG_ENABLE_LED_PWMMODE
    custom_hal_gpio_dap_status_set();
#else
    HAL_GPIO_DAP_STATUS_set();
#endif

  app_dap_event = false;

#ifdef HAL_CONFIG_ENABLE_VCP
  if (app_vcp_event)
#ifdef HAL_CONFIG_ENABLE_LED_PWMMODE
    custom_hal_gpio_vcp_status_toggle();
#else
    HAL_GPIO_VCP_STATUS_toggle();
#endif
  else
#ifdef HAL_CONFIG_ENABLE_LED_PWMMODE
    custom_hal_gpio_vcp_status_write(app_vcp_open);
#else
    HAL_GPIO_VCP_STATUS_write(app_vcp_open);
#endif

  app_vcp_event = false;
#endif
}

//-----------------------------------------------------------------------------
int main(void)
{
  sys_init();
  sys_time_init();
  dap_init();
  usb_init();
#ifdef HAL_CONFIG_ENABLE_VCP
  usb_cdc_init();
#endif
  usb_hid_init();
  serial_number_init();

  app_status_timeout = STATUS_TIMEOUT;

#ifdef HAL_CONFIG_ENABLE_VCP
  HAL_GPIO_VCP_STATUS_out();
  HAL_GPIO_VCP_STATUS_clr();
#endif

  HAL_GPIO_DAP_STATUS_out();
  HAL_GPIO_DAP_STATUS_set();

  HAL_GPIO_BUTTON_in();

//  HAL_GPIO_BUTTON_pullup(); Jeff Probe has an external pullup

  while (1)
  {
    sys_time_task();
    status_timer_task();
#ifdef HAL_CONFIG_ADC_PWRSENSE
    adc_task();
#endif
    usb_task();
    dap_task();

#ifdef HAL_CONFIG_ENABLE_VCP
    tx_task();
    rx_task();
    break_task();
    uart_timer_task();
#endif

#ifdef HAL_CONFIG_ENABLE_BUTTON
    button_task();
#endif
#ifdef HAL_CONFIG_ENABLE_PROVIDE_PWR
    ext_power_task();
#endif
//    if (0 == HAL_GPIO_BOOT_ENTER_read())
//      NVIC_SystemReset();
    if ( reboot_device ) break;
  }

  usb_detach();
  // put bootloader magic and reboot device
  * (uint32_t *) (HMCRAMC0_ADDR+ HMCRAMC0_SIZE-4) = UF2_BOOTLOADER_MAGIC;
  NVIC_SystemReset();
  return 0;
}

