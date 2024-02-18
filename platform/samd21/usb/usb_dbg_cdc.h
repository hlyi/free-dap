#ifndef _USB_DBG_CDC_H_
#define _USB_DBG_CDC_H_

#define DBG_FILL_HEX(var,var_size,msg,msg_loc) for ( int _tmpidx = 0; _tmpidx <var_size; _tmpidx++) { \
		msg[msg_loc- _tmpidx] = "0123456789ABCDEF"[ (var) & 0xf]; \
		var >>=4; }

#endif
