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

// main app API
bool	usb_jeffprobe_is_stop(void);
void	usb_jeffprobe_reboot( void );
void	usb_jeffprobe_set_app_addr(uint32_t *addr, int addr_size);

// usb callback API
bool	usb_jeffprobe_handle_request( usb_request_t *request );

#endif
