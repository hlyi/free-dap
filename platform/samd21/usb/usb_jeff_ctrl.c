#include "usb.h"
#include "usb_dbg_cdc.h"
#include "usb_jeff_ctrl.h"
#include "samd21.h"
#include "hal_gpio.h"
#include "pwr_ctrl.h"

static bool		device_is_stop = false ;
static uint8_t		chainload_idx = 0 ;
static uint32_t	*	reboot_addr = NULL;
static int		reboot_addr_sz = 0;

bool usb_jeffprobe_is_stop()
{
	return device_is_stop;
}

void usb_jeffprobe_set_app_addr (uint32_t *addr, int addr_size)
{
	reboot_addr = addr;
	reboot_addr_sz = addr_size;
}

bool usb_jeffprobe_handle_request(usb_request_t *request)
{
	static alignas(4) uint8_t usb_ret_data[4];
	bool send_zlp = true;
	bool ret_val = true;
	switch(USB_CMD_VALUE(request)){
		case USB_CMD(IN, DEVICE, VENDOR, CMD_PWR_STATUS):
			{
				uint32_t	data = voltage_readout & 0xffffff;
				data |= (power_state) <<30;
				data |= (is_tgt_power_driven() ? 0: 1 ) <<28;
				data |= (HAL_GPIO_nRESET_SENSE_read() ? 0: 1) << 25;
				data |= (HAL_GPIO_nRESET_read()	? 0 : 1 ) << 24;
				*(uint32_t *)usb_ret_data = data;
				usb_control_send(usb_ret_data, 4);
				send_zlp = false;
			}
			break;
		case USB_CMD(OUT, DEVICE, VENDOR, CMD_PWR_OFF):
			set_tgt_power(false);
			break;
		case USB_CMD(OUT, DEVICE, VENDOR, CMD_PWR_ON):
			set_tgt_power(true);
			break;
		case USB_CMD(OUT, DEVICE, VENDOR, CMD_BOOT_BL):
			device_is_stop = true;
			chainload_idx = 0;
			break;
		case USB_CMD(OUT, DEVICE, VENDOR, CMD_BOOT_CL_APP1):
			device_is_stop = true;
			chainload_idx = 1;
			break;
		case USB_CMD(OUT, DEVICE, VENDOR, CMD_BOOT_CL_APP2):
			device_is_stop = true;
			chainload_idx = 2;
			break;
		case USB_CMD(OUT, DEVICE, VENDOR, CMD_NRST_ASSERT):
			HAL_GPIO_nRESET_set();
			break;
		case USB_CMD(OUT, DEVICE, VENDOR, CMD_NRST_DEASSERT):
			HAL_GPIO_nRESET_clr();
			break;
		default:
			ret_val = false;
	}
	if ( ret_val ) {
		if ( send_zlp ) usb_control_send_zlp();
#ifdef JEFF_DEBUG
		static alignas(4) uint8_t	message[18] = {'U','=', 0,0,',',0,0,',',0,0,0,0,0,0,0,0,'\n',0};
		uint32_t dbg = request->bmRequestType;
		DBG_FILL_HEX(dbg, 2, message, 3 );
		dbg = request->bRequest;
		DBG_FILL_HEX(dbg, 2, message, 6 );
		dbg = *(uint32_t*) usb_ret_data;
		DBG_FILL_HEX(dbg, 8, message, 15 );
		usb_cdc_send(message,17);
#endif
	}
	return ret_val;
}

void usb_jeffprobe_reboot()
{
	usb_detach();
	// in case request is out of range, reboot to bootloader
	if ( chainload_idx > reboot_addr_sz ) chainload_idx =  0;

	if ( chainload_idx == 0 ) {
		// put bootloader magic and reboot device
		* (uint32_t *) (HMCRAMC0_ADDR+ HMCRAMC0_SIZE-4) = UF2_BOOTLOADER_MAGIC;
	}else{
		* (uint32_t *) (HMCRAMC0_ADDR+ HMCRAMC0_SIZE-4) = UF2_CHAINLOADER_MAGIC;
		* (uint32_t *) (HMCRAMC0_ADDR+ HMCRAMC0_SIZE-8) = reboot_addr[chainload_idx-1];
	}
	NVIC_SystemReset();
}
