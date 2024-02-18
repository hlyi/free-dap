#include "usb.h"
#include "usb_dbg_cdc.h"
#include "usb_jeff_ctrl.h"
#include "samd21.h"
#include "hal_gpio.h"
#include "pwr_ctrl.h"

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
			reboot_device = true;
			chainload_idx = 0;
			break;
		case USB_CMD(OUT, DEVICE, VENDOR, CMD_BOOT_CL_APP1):
			reboot_device = true;
			chainload_idx = 1;
			break;
		case USB_CMD(OUT, DEVICE, VENDOR, CMD_BOOT_CL_APP2):
			reboot_device = true;
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

void usb_jeff_reboot(uint32_t app1_addr, uint32_t app2_addr)
{
	usb_detach();
	if ( chainload_idx == 0 ) {
		// put bootloader magic and reboot device
		* (uint32_t *) (HMCRAMC0_ADDR+ HMCRAMC0_SIZE-4) = UF2_BOOTLOADER_MAGIC;
	}else{
		* (uint32_t *) (HMCRAMC0_ADDR+ HMCRAMC0_SIZE-4) = UF2_CHAINLOADER_MAGIC;
		* (uint32_t *) (HMCRAMC0_ADDR+ HMCRAMC0_SIZE-8) = (chainload_idx == 2) ? app2_addr : app1_addr;
	}
	NVIC_SystemReset();
}
