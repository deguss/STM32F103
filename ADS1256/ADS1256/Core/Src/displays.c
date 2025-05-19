/*
 * displays.c
 *
 *  Created on: May 6, 2025
 *      Author: Daniel
 */
#include "displays.h"
#include "lcd.h"
//#include "main.h"
#include "stm32f1xx_it.h"
#include "ads1256.h"
#include "gps.h"
#include "file.h"

/*void ITM_SendString(const char* str) {
    while (*str) {
        ITM_SendChar(*str++);
    }
} */



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



void update_display_config(void){
	static char str[21];
	static uint8_t selection;
	static uint8_t callcnt;
	int i;

	lcd_cursor(&lcd, 1, 0);
	if (ADS_state == ADS_READY){
		if (sig_btn != 0){	//pressing BUTTON
			sig_btn=0;
			selection++;
			if (selection==2){ //confirmation
				FRESULT fres = readWriteConfigFile("config.txt", true);	//request to overwrite
				callcnt=10;
				if (fres != FR_OK){
					snprintf(str, sizeof(str), "error saving config");
				}
				else {
					snprintf(str, sizeof(str), "config saved. wait!");
					ADS_state = ADS_INIT;
					stopSampling();
					selection = 0;
				}
				lcd_line(&lcd, str);
			}
			if (selection>=3){
				//ADS_state = ADS_ready;
				//TODO what happens at the second BTN press?
				//clamp at 3
				selection=3;
			}
		}

		if (sig_enc != 0){	//turning encoder
			if (selection==0){	//adjusting SPS
					//int i = getSPSindex(adcDataArray.sps) + sig_enc;
				i = (int)sps_index + sig_enc;
				sps_index = (uint8_t)(MIN(MAX(i,0),(NUM_SPS_OPTIONS-1)));
				adcDataArray[0].sps = sps[sps_index];
				adcDataArray[0].length = bufferSizes[sps_index];
				adcDataArray[1].sps = sps[sps_index];
				adcDataArray[1].length = bufferSizes[sps_index];

			}
			if (selection==1){  //adjusting PGA
				i = (int)pga_index + sig_enc;
				pga_index = (uint8_t)(MIN(MAX(i,0),(NUM_PGA_OPTIONS-1)));
				adcDataArray[0].gain = pga[pga_index];
				adcDataArray[1].gain = pga[pga_index];

			}
			sig_enc = 0;
		}
		//as this function is called with 500ms frequency, wait 10 calls (5s) for letting a further button press stop recording
		if (selection < 2 && !callcnt){	//display "1000Hz 1ch +-5000mV" by default, but not for 5 seconds after config saved
			snprintf(str, sizeof(str), "%5dHz %1dch +-%4dmV", adcDataArray[0].sps, config_channels, range[pga_index]);
			lcd_line(&lcd, str);
		}
		if (callcnt){
			callcnt--;
		}
	}  // end ADS_READY




	else if (ADS_state == ADS_RECORDING){
		if (sig_btn != 0){
			sig_btn=0;
			str[0]='\0';	//clear screen
			selection++;
			if (selection==1){ //stop recording?
				snprintf(str, sizeof(str), "?? stop recording ??");
				callcnt=10;
			}
			if (selection==2 && callcnt){ //stop recording
				selection=0;
				str[0]='\0';	//clear screen
				ADS_state = ADS_INIT;
				stopSampling();
			}
			lcd_line(&lcd, str);
		}
		if (callcnt){
			callcnt--;
			if (!callcnt){
				selection=0;
				str[0]='\0';	//clear screen
				lcd_line(&lcd, str);
			}
		}
	} // end ADS_RECORDING

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
				lcd_string(&lcd, "    ");
				break;
			case ADS_READY:
				if (SD_state == SD_OK)
					lcd_string(&lcd, "ready");
				if (SD_state == SD_FSERROR)
					lcd_string(&lcd, "error");
				break;
			case ADS_RECORDING:
				lcd_string(&lcd, "[REC]");
				break;
			case ADS_ERROR:
				lcd_string(&lcd, "!REC!");
				break;

		}
		ADS_state_past = ADS_state;
	}
	if (USB_state != USB_state_past){
		lcd_cursor(&lcd, 0, 17);
		switch (USB_state){
			case USBD_BUSY:
				lcd_string(&lcd, "   ");
				break;
			case USBD_OK:
				lcd_string(&lcd, "USB");
				break;
			case USBD_FAIL:
				lcd_string(&lcd, "!E!");
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
				//will output usage of SD card [14%]
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
		snprintf(str,20,"[SAT%2d]", sat_in_view);
		lcd_string(&lcd, str);
		sat_in_view_past = sat_in_view;
	}

}
