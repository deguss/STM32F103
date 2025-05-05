#include "main.h"
#include "fatfs.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "ads1256.h"
#include "lcd.h"
#include "stdio.h"
#include "file.h"
#include "gps.h"
#include "core_cm3.h"  // Adjust based on your system





IWDG_HandleTypeDef hiwdg;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim4;

float vdda, c0, c1, c2, c3;

typedef enum {
	ADS_INIT,
	ADS_RECORDING,
	ADS_ERROR,
	ADS_INVALID}  ADS_states;
static ADS_states ADS_state = ADS_INIT;
static USBD_StatusTypeDef USB_state = USBD_BUSY;	//initialize with NOK at beginning
typedef enum {
	SD_INIT,
	SD_OK,
	SD_FSERROR } SD_states;
static SD_states SD_state = SD_INIT;

GPS_states GPS_state = GPS_INIT;
Lcd_HandleTypeDef lcd;

static void updateStates(void);
static void transmitArrayOverUSB(AdcDataArrayStruct *arr);
static void arraytoFile(AdcDataArrayStruct *arr);
static void ITM_Init(void);
void ITM_SendString(const char* str);





static ADS_states startSampling(void){
	//configure differential channel: AIN1 - AIN0
	// test if communication to ADC works (read by register)
	if (setChannel(0,1)){	//if not succeeded
		return ADS_ERROR;
	}

	pga_index = 0; //start with PGA=1
	setGain(getSPSRegValue(adcDataArray.sps), pga_const[pga_index]);	//continuous conversation starts here


	HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 7);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn); //enable DRDY interrupt
	return ADS_RECORDING; //for success
}


static void updateStates(void){
	static ADS_states ADS_state_past = ADS_INVALID;
	static USBD_StatusTypeDef USB_state_past = USBD_FAIL;
	static SD_states SD_state_past = SD_FSERROR;
	static GPS_states GPS_state_past = GPS_ERROR;
	static char str[32];

	if (ADS_state != ADS_state_past){
		lcd_cursor(&lcd, 0, 0);
		switch (ADS_state){
			case ADS_INIT:
				lcd_string(&lcd, "[   ]");
				break;
			case ADS_RECORDING:
				lcd_string(&lcd, "[REC]");
				lcd_cursor(&lcd, 1, 0);
				snprintf(str, 21, "%5dHz %1dch +-%4dmV", adcDataArray.sps, adcDataArray.channels, range[pga_index]);
				lcd_line(&lcd, str);
				break;
			case ADS_ERROR:
				lcd_string(&lcd, "!REC!");
				break;
			case ADS_INVALID:
				lcd_string(&lcd, "?REC?");
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
	if (SD_state != SD_state_past){
		lcd_cursor(&lcd, 0, 5);
		switch (SD_state){
			case SD_INIT:
				lcd_string(&lcd, "[  ]");
				break;
			case SD_OK:
				lcd_string(&lcd, "[SD]");
				break;
			case SD_FSERROR:
				lcd_string(&lcd, "!SD!");
				break;
		}
		SD_state_past = SD_state;
	}
	if (GPS_state != GPS_state_past){
		lcd_cursor(&lcd, 0, 10);
		switch (GPS_state){
			case GPS_INIT:
				lcd_string(&lcd, "[   ]");
				break;
			case GPS_TIME:
				lcd_string(&lcd, "[TIM]");
				break;
			case GPS_FIX:
				lcd_string(&lcd, "[GPS]");
				break;
			case GPS_ERROR:
				lcd_string(&lcd, "!GPS!");
				break;
		}
		GPS_state_past = GPS_state;
	}

}

//-----------------------------------------------------------------------------
int main(void){
	uint32_t percent, total;
	int bytes_written;
	static uint32_t timer_USB, timer_intADC, timer_ADS, timer_mainloop;
	char str[64];
	uint16_t intADCraw[5];
	static float vrefint = 1200;
	//float vdda, c0, c1, c2, c3;
	//float ch0, ch1, ch2, ch3;
	enum display_pages {ADS, INTADC} display_page = ADS;
	char cmd_str[21];
	char query_str[21];
	static int elog = 0;	//indicates whether data is transmitted to host over USB. Initiated by command INIT:ELOG


	HAL_Init();
	SystemClock_Config();
	ITM_Init();
	MX_GPIO_Init();

	HAL_Delay(500);
	ITM_SendString("\r\nSWV (serial wire viewer) test. Stimulus Port 0.0 enabled\r\n");

	Lcd_PortType ports[] = {GPIOB, GPIOB, GPIOB, GPIOB};
	Lcd_PinType pins[] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_8, GPIO_PIN_9};
	lcd = lcd_create(ports, pins, GPIOA, GPIO_PIN_4, GPIOC, GPIO_PIN_13, LCD_4_BIT_MODE);
	lcd_clear(&lcd);
	lcd_line(&lcd, "24-bit data logger");
	lcd_cursor(&lcd, 1, 0);
	lcd_line(&lcd, "Piri Daniel 2025");
	lcd_cursor(&lcd, 2, 0);
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
		lcd_line(&lcd, "watchdog ");
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
		lcd_line(&lcd, "software ");
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
		lcd_line(&lcd, "POR/PDR ");
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
		lcd_line(&lcd, "low power ");
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
		lcd_line(&lcd, "RESET ");
	__HAL_RCC_CLEAR_RESET_FLAGS();

	MX_SPI2_Init(&hspi2);	//interface to SD-card
	MX_FATFS_Init();
	MX_TIM4_Init(&htim4);

	lcd_cursor(&lcd, 3, 0);
	// mount SD card and display usage at start screen
	FRESULT fres = f_mount(&FatFs, "", 1);
	if (fres != FR_OK) {
		// will return FR_NOT_READY: The physical drive cannot work   if no SD card inserted
		// returns FR_NO_FILESYSTEM: There is no valid PRIMARY formatted FAT32 partition on the SD card
		print_fresult(fres, str);
		lcd_line(&lcd, str);
	} else {
		fres = checkSDusage(&percent, &total);
		if (fres != FR_OK){
			print_fresult(fres, str);
			lcd_line(&lcd, str);
		} else {
			snprintf(str, 20, "SD:%2u%% used of %uGB", percent, total);
			ITM_SendString(str);
			ITM_SendChar('\n');
			lcd_line(&lcd, str);
		}
	}

	HAL_Delay(3000);

	char value[64]; // Buffer to store the value
	char uid[25];
	getSTM_UID(uid);
	snprintf(str, sizeof(str),"MCU UID = %s\n", uid);
	ITM_SendString(str);

	//check if file present and UID matches
	fres = read_config("UID", value, sizeof(value));
	if (fres != FR_OK  || memcmp(uid, value, strlen(uid)) != 0) {
		ITM_SendString("config.txt not present or wrong UID!\n");
		fres = write_config("UID", uid);
		if (fres == FR_OK)
			fres = write_config("CH", "1");
		if (fres == FR_OK)
			fres = write_config("SPS", "100");
		if (fres != FR_OK){
			ITM_SendString("failed to create config file\n");
		}
	}

	ITM_SendString("config file read\n");
	fres = read_config("CH", value, sizeof(value));
	if (fres == FR_OK){
		adcDataArray.channels = check_range(atoi(value), 1, 2);
		ITM_SendString("CH=");
		snprintf(str, sizeof(str), "%u\n", adcDataArray.channels);
		ITM_SendString(str);
	}
	fres = read_config("SPS", value, sizeof(value));
	if (fres == FR_OK){
		uint16_t index = getSPSindex(validateSPS(atoi(value)));
		adcDataArray.sps = sps[index];
		adcDataArray.length = bufferSizes[index];
		ITM_SendString("SPS=");
		snprintf(str, sizeof(str), "%u\n", adcDataArray.sps);
		ITM_SendString(str);
		SD_state = SD_OK;
	}


	// Inits: USB, int. ADC
	MX_USB_DEVICE_Init();
	MX_DMA_Init();
	MX_ADC1_Init(&hadc1);
	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK){
		lcd_clear(&lcd);
		lcd_cursor(&lcd, 1, 0);
		lcd_line(&lcd, "internal ADC calibr. failed!");
		Error_Handler();
	}
	if ( HAL_ADC_Start_DMA(&hadc1, (uint32_t*)(intADCraw), hadc1.Init.NbrOfConversion) ){
		lcd_clear(&lcd);
		lcd_line(&lcd, "ADC DMA setup failed!");
		Error_Handler();
	}
	MX_SPI1_Init(&hspi1);	//interface to ADS1256
	//MX_I2C_Init(&hi2c1);
	MX_USART1_UART_Init(&huart1); //initializes UART RX IT for GPS

	uint8_t status;
	status = setupADS1256();	//should be 0x30 for this particular piece of AD1256 (ID)

/*
	// Enable PVD with the desired voltage threshold
	PWR_PVDTypeDef sConfigPVD;
	sConfigPVD.PVDLevel = PWR_PVDLEVEL_7; // Set PVD detection level (e.g., 2.9V)
	sConfigPVD.Mode = PWR_PVD_MODE_IT_FALLING; // Set rising edge interrupt mode

	HAL_PWR_ConfigPVD(&sConfigPVD);
	HAL_PWR_EnablePVD();

	// Enable PVD EXTI interrupt in NVIC
	HAL_NVIC_SetPriority(PVD_IRQn, 0, 0);  //highest priority interrupt to unmount SD card in case of a power loss
	HAL_NVIC_EnableIRQ(PVD_IRQn);
*/

	//MX_IWDG_Init();
	//__HAL_DBGMCU_FREEZE_IWDG();

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);	// Receive Not Empty interrupt from GPS
	HAL_NVIC_SetPriority(USART1_IRQn, 14, 7);  //highest priority interrupt to unmount SD card in case of a power loss
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	// Start the first receive operation with interrupt
	//HAL_UART_RxCpltCallback(&huart1);
	uint8_t rx_data; // Variable to store the received byte
    if (HAL_UART_Receive_IT(&huart1, &rx_data, 1) != HAL_OK){
    	ITM_SendString("UART (GPS) seems not to be configured properly.");
    }


	timer_USB = HAL_GetTick();	//start timers
	timer_ADS = HAL_GetTick();
	timer_intADC = HAL_GetTick();
	lcd_clear(&lcd);

	//----------------------------------------------------------------------------------------
	while (1){
	//---------------------------------------------------------------------------------------

		timer_mainloop = HAL_GetTick();

		// -------------- interprete commands received over USB --------------------
		if (usb_received){
#ifdef DEBUG
			ITM_SendString("USB_received triggered: ");
			lcd_cursor(&lcd, 3,0);
			snprintf(str, usb_received+1, "%s", UserRxBufferFS);
			ITM_SendString(str);
			ITM_SendString("\n");
			lcd_line(&lcd, str);
#endif

			strcpy(cmd_str, (char *)(UserRxBufferFS)); //null terminate string
			memset(UserRxBufferFS, 0, APP_RX_DATA_SIZE);
			size_t char_length = strlen(cmd_str);
			char last_char = cmd_str[char_length-1];

			if (strncmp((const char *)("*RST"), cmd_str, 4) == 0){
				NVIC_SystemReset();
			}

			if (strncmp((const char *)("INIT:DLOG"), cmd_str, 9) == 0){
				SD_state = SD_OK;
			}
			if (strncmp((const char *)("ABOR:DLOG"), cmd_str, 9) == 0){
				SD_state = SD_INIT;
			}

			if (strncmp((const char *)("INIT:ELOG"), cmd_str, 9) == 0){
				elog = 1;
			}
			if (strncmp((const char *)("ABOR:ELOG"), cmd_str, 9) == 0){
				elog = false;
			}

			if (strncmp((const char *)("CONF:CHAN"), cmd_str, 9) == 0){
				snprintf(str, sizeof(str), "query %s received\n", cmd_str);
				ITM_SendString(str);
				if (last_char == '?'){
					size_t len = snprintf(query_str, sizeof(query_str), "%u", adcDataArray.channels);
					CDC_Transmit_FS((uint8_t *)query_str, len);
					snprintf(str, sizeof(str),"responded: %s\n", query_str);
					ITM_SendString(str);
				}
				else{
					ADS_state = ADS_INIT;
					sscanf(cmd_str, "CONF:CHAN %hhu", &adcDataArray.channels);
					snprintf(str, sizeof(str), "command %s received. Channel set to %u\n", cmd_str, adcDataArray.channels);
					ITM_SendString(str);
				}

			}
			if (strncmp((const char *)("CONF:SPS"), cmd_str, 8) == 0){
				snprintf(str, sizeof(str), "query %s received\n", cmd_str);
				ITM_SendString(str);
				if (last_char == '?'){
					size_t len = snprintf(query_str, sizeof(query_str), "%u", adcDataArray.sps);
					CDC_Transmit_FS((uint8_t *)query_str, len);
					snprintf(str, sizeof(str),"responded: %s\n", query_str);
					ITM_SendString(str);
				}
				else{
					ADS_state = ADS_INIT;
					uint16_t value, index;
					sscanf(cmd_str, "CONF:SPS %hu", &value);
					index = getSPSindex(validateSPS(value));
					adcDataArray.sps = sps[index];
					adcDataArray.length = bufferSizes[index];
					snprintf(str, sizeof(str), "SPS set to %u\n", adcDataArray.sps);
					ITM_SendString(str);
				}

			}
			lcd_cursor(&lcd, 1, 0);
			snprintf(str, 21, "%5dHz %1dch +-%4dmV", adcDataArray.sps, adcDataArray.channels, range[pga_index]);
			lcd_line(&lcd, str);

			usb_received = 0;
		} // end of interpreting commands received over USB --------------------

		if (dma_complete){
			dma_complete=0;
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)(intADCraw), hadc1.Init.NbrOfConversion);
			vdda = vrefint / (float)(intADCraw[4]);	//calculate actual VDD based on VREFINT measurement
			if (display_page == INTADC && HAL_GetTick() - timer_intADC > 300){ //check if 300ms ellapsed
				timer_intADC = HAL_GetTick();	//update timer with present value of time
				lcd_cursor(&lcd, 1,0);
				snprintf(str, 20, "VDDA = %04imV", (int)(vdda*4095.0));
				lcd_line(&lcd, str);
				if (vdda > 0.9 || vdda < 0.7){
					lcd_cursor(&lcd, 2,0);
					lcd_line(&lcd, "ADC seems defective");
				}
				else {
					c0 = vdda*(float)(intADCraw[0]);
					c1 = vdda*(float)(intADCraw[1]);
					c2 = vdda*(float)(intADCraw[2]);
					c3 = vdda*(float)(intADCraw[3]);
					lcd_cursor(&lcd, 2,0);
					snprintf(str, 20, "A0:%04imV A1:%04imV", (int)(c0), (int)(c1));
					lcd_line(&lcd, str);
					lcd_cursor(&lcd, 3,0);
					snprintf(str, 20, "A2:%04imV A3:%04imV", (int)(c2), (int)(c3));
					lcd_line(&lcd, str);
				}
			}

		}
		if (display_page == ADS && ADS_state == ADS_RECORDING && HAL_GetTick() - timer_ADS > 500){ //check if 300ms ellapsed
			timer_ADS = HAL_GetTick();	//update timer with present value of time

			int32_t ch0 = calculate_average(adcDataArray.data, adcDataArray.length);
			lcd_cursor(&lcd, 2,0);
			uint8_t decimals=2;
			int32_t abs_ch0 = (ch0 < 0) ? -ch0 : ch0; // Calculate absolute value
			if (abs_ch0 < 1e6) //<1000mV
			    decimals = 3;
			if (abs_ch0 < 1e5) //<100mV
			    decimals = 4;
			if (abs_ch0 < 1e4)  //<10mV
			    decimals = 5;

			//snprintf(str, 20, "A:% 8.*fmV        ",decimals, ch0);
			// Format the output string in integer math.
			snprintf(str, 20, "A:%d.%0*dmV", ch0/1000, decimals, abs_ch0 % 1000);
			lcd_line(&lcd, str);
/*			lcd_cursor(&lcd, 3,0);
			snprintf(str, 20, "%3u", encoder_count);
			lcd_line(&lcd, str);
*/
		}

		if (ADS_state == ADS_INIT) {
			status = setupADS1256();
			lcd_cursor(&lcd, 2,0);
			snprintf(str, 20, "ADS1256 status: 0x%02X", status);
			lcd_line(&lcd, str);
			ADS_state = startSampling();
		}
		if (ADS_state == ADS_RECORDING && flagBufferFull) { //ADC receive ring buffer full
			flagBufferFull = 0;
			if (USB_state == USBD_OK && elog){
				transmitArrayOverUSB(&adcDataArray);
			}
			if (SD_state == SD_OK){
				arraytoFile(&adcDataArray);
			}
		}

		//if (GPS_state == GPS_INIT){
		parse_gprmc_datetime(GPS_rx_buf, str, &hours, &minutes, &seconds, &GPS_state);
		lcd_cursor(&lcd, 3,0);
		lcd_line(&lcd, str);




		if (USB_state ==  USBD_OK){
			;
		}
		else { //if USB status not OK
			if (HAL_GetTick() - timer_USB > 1000){ //check if 1s ellapsed
				timer_USB = HAL_GetTick();	//update timer with present value of time
				USB_state = CDC_Transmit_FS(&status, 1);	//transmit some msg
			}
		}
		if (ADS_state == ADS_ERROR) {
			if (HAL_GetTick() - timer_ADS > 5000){ //check if 5s ellapsed
				lcd_cursor(&lcd, 2,0);
				lcd_line(&lcd, "please power cycle! ");
				timer_ADS = HAL_GetTick();	//update timer with present value of time
				HAL_NVIC_DisableIRQ(EXTI0_IRQn); //disable DRDY interrupt
				stopSampling();
				ADS_state = ADS_INIT;
			}
		}


		updateStates();
		//HAL_IWDG_Refresh(&hiwdg);
	}

}



// Function to transmit variable length int32_t array over USB CDC
static void transmitArrayOverUSB(AdcDataArrayStruct *arr){
/*typedef struct {
    uint16_t length;
    uint16_t sps;
    uint32_t time;
    uint16_t res16;
    uint8_t  res8;
    uint8_t channels;
    int32_t data[ADCBUFLEN];
} AdcDataArrayStruct;*/

    // Calculate the total size of the data to be sent
    size_t totalSize = 2*2+4+2+1+1 + arr->length*sizeof(int32_t);

    // Transmit the data over USB CDC
    CDC_Transmit_FS((uint8_t *)arr, totalSize);
}

static void arraytoFile(AdcDataArrayStruct *arr){
	;
}


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

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
