// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024, Alex Taradov <alex@taradov.com>. All rights reserved.

#ifndef _HAL_CONFIG_H_
#define _HAL_CONFIG_H_

/*- Includes ----------------------------------------------------------------*/
#include "samd21.h"
#include "hal_gpio.h"

/*- Definitions -------------------------------------------------------------*/
//#define HAL_BOARD_GENERIC

#if defined(HAL_BOARD_CUSTOM)
  // Externally supplied board configuration takes precedence
  #include HAL_BOARD_CUSTOM

#elif defined(HAL_BOARD_GENERIC)
  #define HAL_CONFIG_ENABLE_VCP
  #define DAP_CONFIG_ENABLE_JTAG

  HAL_GPIO_PIN(SWCLK_TCK,          B, 0)
  HAL_GPIO_PIN(SWDIO_TMS,          B, 1)
  HAL_GPIO_PIN(TDI,                B, 2)
  HAL_GPIO_PIN(TDO,                B, 3)
  HAL_GPIO_PIN(nRESET,             B, 4)

  HAL_GPIO_PIN(VCP_STATUS,         A, 10);
  HAL_GPIO_PIN(DAP_STATUS,         B, 30);
  HAL_GPIO_PIN(BOOT_ENTER,         A, 31);

  HAL_GPIO_PIN(UART_TX,            A, 4);
  HAL_GPIO_PIN(UART_RX,            A, 5);

  #define UART_SERCOM              SERCOM0
  #define UART_SERCOM_PMUX         PORT_PMUX_PMUXE_D_Val
  #define UART_SERCOM_GCLK_ID      SERCOM0_GCLK_ID_CORE
  #define UART_SERCOM_APBCMASK     PM_APBCMASK_SERCOM0
  #define UART_SERCOM_IRQ_INDEX    SERCOM0_IRQn
  #define UART_SERCOM_IRQ_HANDLER  irq_handler_sercom0
  #define UART_SERCOM_TXPO         0 // PAD[0]
  #define UART_SERCOM_RXPO         1 // PAD[1]

#elif defined(HAL_BOARD_JEFF_PROBE)
  #define HAL_CONFIG_ENABLE_VCP
  #define HAL_CONFIG_ENABLE_PWR_LED
  #define HAL_CONFIG_ENABLE_LED_PWMMODE
  #define HAL_CONFIG_ENABLE_BUTTON
  #define HAL_CONFIG_ENABLE_PROVIDE_PWR
  #define HAL_CONFIG_ENABLE_USB_VEN
  #define DAP_CONFIG_ENABLE_JTAG
  #define DAP_CONFIG_ENABLE_TMS_DIR
  #define DAP_CONFIG_ENABLE_RST_SENSE
  #define DAP_CONFIG_RST_ACTIVE_HIGH

  HAL_GPIO_PIN(SWCLK_TCK,          A, 17)	// Mod with PA17, orginal control is on PA06
  HAL_GPIO_PIN(SWDIO_TMS,          A, 0)
  HAL_GPIO_PIN(SWDIO_TMS_DIR,      A, 15)
  HAL_GPIO_PIN(TDI,                A, 16)
  HAL_GPIO_PIN(TDO,                A, 19)
  HAL_GPIO_PIN(nRESET,             A, 8)
  HAL_GPIO_PIN(nRESET_SENSE,       A, 9)
  HAL_GPIO_PIN(EXT_PWR,            A, 1);	// Mod with PA01 for PWM control. original control is on PA28

  HAL_GPIO_PIN(UART_TX,            A, 4);	// PAD[0]
  HAL_GPIO_PIN(UART_RX,            A, 7);	// PAD[3]

  HAL_GPIO_PIN(VCP_STATUS,         A, 10);
  HAL_GPIO_PIN(DAP_STATUS,         A, 11);
  HAL_GPIO_PIN(PWR_STATUS,         A, 14);
  HAL_GPIO_PIN(BUTTON,             A, 27);

  /* PA10/WO[2], PA11/WO[3], PA14/WO[4] */
  // CC0 used by Power LED
  #define POWER_LED_CC_CH		0
  // CC2 used by VCP
  #define VCP_STATUS_CC_CH		2
  // CC3 used by DAP
  #define DAP_STATUS_CC_CH		3

  #define HAL_CONFIG_ADC_PWRSENSE
  HAL_GPIO_PIN(ADC_PWRSENSE,          A, 2)
  HAL_GPIO_PIN(ADC_VREF,              A, 3)

  #define UART_SERCOM              SERCOM0
  #define UART_SERCOM_PMUX         PORT_PMUX_PMUXE_D_Val
  #define UART_SERCOM_GCLK_ID      SERCOM0_GCLK_ID_CORE
  #define UART_SERCOM_APBCMASK     PM_APBCMASK_SERCOM0
  #define UART_SERCOM_IRQ_INDEX    SERCOM0_IRQn
  #define UART_SERCOM_IRQ_HANDLER  irq_handler_sercom0
  #define UART_SERCOM_TXPO         0
  #define UART_SERCOM_RXPO         3

#else
  #error No board defined
#endif

#endif // _HAL_CONFIG_H_

