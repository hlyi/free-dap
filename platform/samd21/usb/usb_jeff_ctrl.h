#ifndef _USB_JEFF_CTRL_H_
#define _USB_JEFF_CTRL_H_

#include "usb.h"

//#define JEFF_DEBUG

#define UF2_BOOTLOADER_MAGIC    0xf01669ef
#define UF2_CHAINLOADER_MAGIC   0xf04669ef

#define USB_CMD_PWR_STATUS      0x51
#define USB_CMD_PWR_OFF         0x52
#define USB_CMD_PWR_ON          0x53
#define USB_CMD_NRST_DEASSERT   0x54
#define USB_CMD_NRST_ASSERT     0x55
#define USB_CMD_BOOT_BL         0x62
#define USB_CMD_BOOT_CL_APP1    0x63
#define USB_CMD_BOOT_CL_APP2    0x64

extern volatile bool	reboot_device ;
extern volatile uint8_t	chainload_idx ;
extern uint32_t		voltage_readout;

bool			usb_jeffprobe_handle_request( usb_request_t *request );
void			usb_jeff_reboot( uint32_t app1_addr, uint32_t app2_addr );

#endif
