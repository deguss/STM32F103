/*
 * displays.c
 *
 *  Created on: May 6, 2025
 *      Author: Daniel
 */
#include "displays.h"


//needed for printf ITM debug output
int _write(int file, char *ptr, int len){
	for (int i=0; i<len; i++){
		ITM_SendChar(*ptr++);
	}
	return len;
}

void ITM_Init(void) {
    // Check if ITM is present and enabled
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0) {
        ITM->TCR = ITM_TCR_ITMENA_Msk; // Enable ITM
        ITM->TER |= (1UL << 0); // Enable stimulus port 0
    }
}


void ITM_SendString(const char* str) {
    while (*str) {
        ITM_SendChar(*str++);
    }
}

void updateStatesLCD(uint32_t percentFull, uint8_t sat_in_view){
	static ADS_states ADS_state_past = ADS_ERROR;
	static USBD_StatusTypeDef USB_state_past = USBD_FAIL;
	static SD_states SD_state_past = SD_FSERROR;
	//static GPS_states GPS_state_past = GPS_ERROR;
	static char str[25];
	static uint8_t sat_in_view_past = 12;
	static uint32_t percentFull_old=100;

	if (ADS_state != ADS_state_past){
		lcd_cursor(&lcd, 0, 0);
		switch (ADS_state){
			case ADS_INIT:
				lcd_string(&lcd, "[   ]");
				break;
			case ADS_READY:
				lcd_string(&lcd, "ready");
				break;
			case ADS_RECORDING:
				lcd_string(&lcd, "[REC]");
				//lcd_cursor(&lcd, 1, 0);
				//snprintf(str, sizeof(str), "%5dHz %1dch +-%4dmV", adcDataArray.sps, config_channels, range[pga_index]);
				//lcd_line(&lcd, str);
				break;
			case ADS_ERROR:
				lcd_string(&lcd, "!REC!");
				break;

		}
		ADS_state_past = ADS_state;
	}
	if (USB_state != USB_state_past){
		lcd_cursor(&lcd, 0, 15);
		switch (USB_state){
			case USBD_BUSY:
				lcd_string(&lcd, "[   ]");
				break;
			case USBD_OK:
				lcd_string(&lcd, "[USB]");
				break;
			case USBD_FAIL:
				lcd_string(&lcd, "!USB!");
				break;
		}
		USB_state_past = USB_state;
	}
	if (SD_state != SD_state_past || percentFull_old != percentFull){
		lcd_cursor(&lcd, 0, 5);
		switch (SD_state){
			case SD_INIT:
				lcd_string(&lcd, "[  ] ");
				break;
			case SD_OK:
				char str[6]; //will output usage of SD card [14%]
				snprintf(str, sizeof(str), "[%2lu%%]", percentFull);
				lcd_string(&lcd, str);
				break;
			case SD_FSERROR:
				lcd_string(&lcd, "!SD! ");
				break;
		}
		SD_state_past = SD_state;
		percentFull_old = percentFull;
	}
	if (sat_in_view != sat_in_view_past){
		lcd_cursor(&lcd, 0, 10);
		//switch (GPS_state){
			/*case GPS_INIT:
				lcd_string(&lcd, "NO GPS");
				break;
			case GPS_TIME:*/
				snprintf(str,20,"SAT%2d", sat_in_view);
				lcd_string(&lcd, str);
			/*	break;
			case GPS_FIX:
				lcd_string(&lcd, "[GPS]");
				break;
			case GPS_ERROR:
				lcd_string(&lcd, "!GPS!");
				break;
		}*/
		sat_in_view_past = sat_in_view;
	}

}
