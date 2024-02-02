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
#include "button.h"
#include "pwr_ctrl.h"
#include "led_status.h"

/*- Definitions -------------------------------------------------------------*/
#define USB_BUFFER_SIZE		64
#define UART_WAIT_TIMEOUT	10	// ms

/*- Variables ---------------------------------------------------------------*/
static alignas(4) uint8_t app_req_buf_hid[DAP_CONFIG_PACKET_SIZE];
static alignas(4) uint8_t app_req_buf_bulk[DAP_CONFIG_PACKET_SIZE];
static alignas(4) uint8_t app_req_buf[DAP_CONFIG_PACKET_SIZE];
static alignas(4) uint8_t app_resp_buf[DAP_CONFIG_PACKET_SIZE];
static int app_req_buf_hid_size = 0;
static int app_req_buf_bulk_size = 0;
static bool app_resp_free = true;
uint64_t app_system_time = 0;
bool app_event = false;

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
bool vcp_event = false;
bool vcp_opened = false;
#endif


//#define BUTTON_DEBUG
//#define ADC_DEBUG
//#define PWR_DEBUG
//#define USBVEN_DEBUG

#ifdef HAL_CONFIG_ENABLE_USB_VEN
#define USB_CMD_PWR_STATUS      0x51
#define USB_CMD_PWR_OFF         0x52
#define USB_CMD_PWR_ON          0x53
#define USB_CMD_NRST_DEASSERT   0x54
#define USB_CMD_NRST_ASSERT     0x55
#define USB_CMD_BOOT_BL         0x62
#define USB_CMD_BOOT_CL_APP1    0x63
#define USB_CMD_BOOT_CL_APP2    0x64

#define CHAINLOAD_APP1_ADDR     0x10000
#define CHAINLOAD_APP2_ADDR     0x18000

#define UF2_BOOTLOADER_MAGIC    0xf01669ef
#define UF2_CHAINLOADER_MAGIC   0xf04669ef
static bool     reboot_device = false;
static uint8_t  chainload_idx = 0;
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
      data |= (power_state) <<30;
      data |= (is_tgt_power_driven() ? 0: 1 ) <<28;
      data |= (HAL_GPIO_nRESET_SENSE_read() ? 0: 1) << 25;
      data |= (HAL_GPIO_nRESET_read()  ? 0 : 1 ) << 24;
      *(uint32_t *)usb_ret_data = data;
      usb_control_send(usb_ret_data, 4);
      send_zlp = false;
    }
  }else {
    // out request
    switch ( data[1] ){
    case USB_CMD_PWR_OFF:
      set_tgt_power(false);
      break;
    case USB_CMD_PWR_ON:
      set_tgt_power(true);
      break;
    case USB_CMD_BOOT_BL:
      reboot_device = true;
      chainload_idx = 0;
      break;
    case USB_CMD_BOOT_CL_APP1:
      reboot_device = true;
      chainload_idx = 1;
      break;
    case USB_CMD_BOOT_CL_APP2:
      reboot_device = true;
      chainload_idx = 2;
      break;
    case USB_CMD_NRST_ASSERT:
      HAL_GPIO_nRESET_set();
      break;
    case USB_CMD_NRST_DEASSERT:
      HAL_GPIO_nRESET_clr();
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



static void custom_init(void)
{
#ifdef HAL_CONFIG_ENABLE_USB_VEN
  usb_setup_recv(usb_ven_setup_callback);
#endif
#ifdef DAP_CONFIG_ENABLE_RST_SENSE
  HAL_GPIO_nRESET_SENSE_in();
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
    vcp_event = true;

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
    vcp_event = true;

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

  vcp_opened        = status;
  app_send_buffer_ptr = 0;
  app_uart_timeout    = 0;
  app_break_timeout   = 0;

  if (vcp_opened)
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
  app_event = true;
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
  pwr_ctrl_init();
  button_init();
  led_status_init();

  while (!reboot_device)
  {
    sys_time_task();
    usb_task();
    dap_task();
    pwr_ctrl_task();
    led_status_task();
    button_task();

#ifdef HAL_CONFIG_ENABLE_VCP
    tx_task();
    rx_task();
    break_task();
    uart_timer_task();
#endif

//    if (0 == HAL_GPIO_BOOT_ENTER_read())
//      NVIC_SystemReset();
  }

  usb_detach();
  if ( chainload_idx == 0 ) {
    // put bootloader magic and reboot device
    * (uint32_t *) (HMCRAMC0_ADDR+ HMCRAMC0_SIZE-4) = UF2_BOOTLOADER_MAGIC;
  }else{
    * (uint32_t *) (HMCRAMC0_ADDR+ HMCRAMC0_SIZE-4) = UF2_CHAINLOADER_MAGIC;
    * (uint32_t *) (HMCRAMC0_ADDR+ HMCRAMC0_SIZE-8) = (chainload_idx == 2) ? CHAINLOAD_APP2_ADDR : CHAINLOAD_APP1_ADDR;
  }
  NVIC_SystemReset();
  return 0;
}

