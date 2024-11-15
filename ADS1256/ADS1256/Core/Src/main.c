#include "main.h"
#include "fatfs.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "ads1256.h"
#include "lcd.h"
#include "stdio.h"
#include "file.h"



IWDG_HandleTypeDef hiwdg;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim4;

float ch0, ch1, ch2, ch3;
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
typedef enum {
	GPS_INIT,
	GPS_TIME,
	GPS_FIX,
	GPS_ERROR } GPS_states;
static GPS_states GPS_state = GPS_INIT;
Lcd_HandleTypeDef lcd;

static void updateStates(void);
static void transmitArrayOverUSB(AdcDataArrayStruct *arr);
static void arraytoFile(AdcDataArrayStruct *arr);

void print_fresult(FRESULT res);



static int dma_complete = 0;
static uint8_t spsi = 8;	//start with 100Hz sampling rate



static ADS_states startSampling(uint8_t sps_index){
	//configure differential channel: AIN1 - AIN0
	// test if communication to ADC works (read by register)
	if (setChannel(0,1)){	//if not succeeded
		return ADS_ERROR;
	}

	pga_index = 0; //start with PGA=1
	setGain(sps_const[sps_index], pga_const[pga_index]);	//continuous conversation starts here

	adcDataArray.sps = sps[sps_index];
	adcDataArray.length = bufferSizes[sps_index];
	adcDataArray.channels = 1;

	HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 7);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn); //enable DRDY interrupt
	return ADS_RECORDING; //for success
}

/*
static void keepBlueLEDFlashing(void){
	// produce a non-blocking 1s OFF and 50ms ON timer (heartbeat signal green LED)
	static bool initialized = false;
	static uint32_t tick_s, tick_r;

	if (!initialized) {
		tick_s = HAL_GetTick();
		tick_r = HAL_GetTick();
		initialized = true;
	}


	if (HAL_GetTick() - tick_s == 1000  &&  !get_BLED()){
		//TODO: tick_s may overflow in 49 days
		tick_r = HAL_GetTick();	//start reset timer
		BLED(1)
	}
	if (HAL_GetTick() - tick_r == 50 && get_BLED()){
		tick_s = HAL_GetTick(); //start set timer
		BLED(0)
	}

}
*/

static void updateStates(void){
	static ADS_states ADS_state_past = ADS_INVALID;
	static USBD_StatusTypeDef USB_state_past = USBD_FAIL;
	static SD_states SD_state_past = SD_FSERROR;
	static GPS_states GPS_state_past = GPS_ERROR;
	static char str[24];

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
				lcd_string(&lcd, str);
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
	static uint32_t timer_USB, timer_intADC, timer_ADS, timer_mainloop;
	static char str[21];
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
	MX_GPIO_Init();

	HAL_Delay(500);
	printf("\r\nSWV (serial wire viewer) test. Stimulus Port 0.0 enabled\r\n");

	Lcd_PortType ports[] = {GPIOB, GPIOB, GPIOB, GPIOB};
	Lcd_PinType pins[] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_8, GPIO_PIN_9};
	lcd = lcd_create(ports, pins, GPIOA, GPIO_PIN_4, GPIOC, GPIO_PIN_13, LCD_4_BIT_MODE);
	lcd_clear(&lcd);
	lcd_string(&lcd, "24-bit data logger  ");
	lcd_cursor(&lcd, 1, 0);
	lcd_string(&lcd, "Piri Daniel 2024");
	lcd_cursor(&lcd, 2, 0);
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
		lcd_string(&lcd, "watchdog ");
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
		lcd_string(&lcd, "software ");
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
		lcd_string(&lcd, "POR/PDR ");
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
				lcd_string(&lcd, "low power ");
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
		lcd_string(&lcd, "RESET ");
	__HAL_RCC_CLEAR_RESET_FLAGS();

	MX_SPI2_Init(&hspi2);	//interface to SD-card
	MX_FATFS_Init();
	MX_TIM4_Init(&htim4);

	lcd_cursor(&lcd, 3, 0);
	FRESULT fres = mountSD();
	if (fres != FR_OK) {
		sprintf(str, "SD-CARD ERROR: %2d",fres);
		lcd_string(&lcd, str);
	} else {
		fres = checkSDusage(&percent, &total);
		snprintf(str, 20, "SD:%2u%% used of %2uGB", percent, total);
		printf("%s\n",str);
		lcd_string(&lcd, str);
	}

	HAL_Delay(3000);

	MX_USB_DEVICE_Init();
	MX_DMA_Init();
	MX_ADC1_Init(&hadc1);
	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK){
		lcd_clear(&lcd);
		lcd_cursor(&lcd, 1, 0);
		lcd_string(&lcd, "internal ADC calibr. failed!");
		Error_Handler();
	}

	if ( HAL_ADC_Start_DMA(&hadc1, (uint32_t*)(intADCraw), hadc1.Init.NbrOfConversion) ){
		lcd_clear(&lcd);
		lcd_string(&lcd, "ADC DMA setup failed!");
		Error_Handler();
	}


	MX_SPI1_Init(&hspi1);	//interface to ADS1256
	//MX_I2C_Init(&hi2c1);
	//MX_USART1_UART_Init(&huart1);
	uint8_t status;
	status = setupADS1256();	//should be 0x30 for this particular piece of AD1256 (ID)


	timer_USB = HAL_GetTick();	//start timers
	timer_ADS = HAL_GetTick();
	timer_intADC = HAL_GetTick();

	//MX_IWDG_Init();
	//__HAL_DBGMCU_FREEZE_IWDG();


	while(1) ;
	lcd_clear(&lcd);

	fres = f_open(&fil, "test.txt", FA_CREATE_ALWAYS | FA_WRITE);
	if (fres != FR_OK) {
		print_fresult(fres);
		sprintf(str, "f_open: %2d",fres);
		lcd_string(&lcd, str);
	}


	while(1){
		HAL_Delay(5000);
		NVIC_SystemReset();
	}

	while (1){
		timer_mainloop = HAL_GetTick();

		// -------------- interprete commands received over USB --------------------
		if (usb_received){
#ifdef DEBUG
			printf("USB_received triggered\n");
			lcd_cursor(&lcd, 3,0);
			snprintf(str, usb_received+1, "%s", UserRxBufferFS);
			lcd_string(&lcd, str);
#endif

			strcpy(cmd_str, (char *)(UserRxBufferFS)); //null terminate string
			size_t char_length = strlen(cmd_str);
			char last_char = cmd_str[char_length-1];

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
				SD_state = SD_INIT;
				ADS_state = ADS_INIT;
				sscanf(cmd_str, "CONF:CHAN %hhu", &adcDataArray.channels);
				printf("command %s received. Channel set to %u\n", cmd_str, adcDataArray.channels);

			}
			if (strncmp((const char *)("CONF:SPSI"), cmd_str, 9) == 0){
				if (last_char == '?'){
					printf("query %s received.\n", cmd_str);
					size_t len = snprintf(query_str, sizeof(query_str), "%d", adcDataArray.sps);
					CDC_Transmit_FS((uint8_t *)query_str, len);
					printf("responded: %s", query_str);
				}
				else{
					SD_state = SD_INIT;
					ADS_state = ADS_INIT;
					sscanf(cmd_str, "CONF:SPSI %hu", &spsi);
					printf("command %s received. SPSI set to %u\n", cmd_str, spsi);
				}

			}
			lcd_cursor(&lcd, 1, 0);
			snprintf(str, 21, "%5dHz %1dch +-%4dmV", adcDataArray.sps, adcDataArray.channels, range[pga_index]);
			lcd_string(&lcd, str);

			usb_received = 0;
		} // end of interpreting commands received over USB --------------------

		if (dma_complete){
			dma_complete=0;
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)(intADCraw), hadc1.Init.NbrOfConversion);
			vdda = vrefint / (float)(intADCraw[4]);	//calculate actual VDD based on VREFINT measurement
			if (display_page == INTADC && HAL_GetTick() - timer_intADC > 300){ //check if 300ms ellapsed
				timer_intADC = HAL_GetTick();	//update timer with present value of time
				lcd_cursor(&lcd, 1,0);
				sprintf(str, "VDDA = %04imV ", (int)(vdda*4095.0));
				lcd_string(&lcd, str);
				if (vdda > 0.9 || vdda < 0.7){
					lcd_cursor(&lcd, 2,0);
					lcd_string(&lcd, "ADC seems defective");
				}
				else {
					c0 = vdda*(float)(intADCraw[0]);
					c1 = vdda*(float)(intADCraw[1]);
					c2 = vdda*(float)(intADCraw[2]);
					c3 = vdda*(float)(intADCraw[3]);
					lcd_cursor(&lcd, 2,0);
					sprintf(str, "A0:%04imV A1:%04imV", (int)(c0), (int)(c1));
					lcd_string(&lcd, str);
					lcd_cursor(&lcd, 3,0);
					sprintf(str, "A2:%04imV A3:%04imV", (int)(c2), (int)(c3));
					lcd_string(&lcd, str);
				}
			}

		}
		if (display_page == ADS && ADS_state == ADS_RECORDING && HAL_GetTick() - timer_ADS > 500){ //check if 300ms ellapsed
			timer_ADS = HAL_GetTick();	//update timer with present value of time
			//calculate average
			int32_t sum=0;
			for (int i=0; i<adcDataArray.length; i++){
				sum+=adcDataArray.data[i];
			}
			ch0 = (float)(sum)/(float)(adcDataArray.length) / 0x800000 * 5000.0;	// /2^23 * 5000mV
			lcd_cursor(&lcd, 2,0);
			//sprintf(str, "A:% 8.2fmV        ", ch0);
			uint8_t decimals=2;
			if (abs(ch0) < 1000)
				decimals = 3;
			if (abs(ch0) < 100)
				decimals = 4;
			if (abs(ch0) < 10)
				decimals = 5;
			sprintf(str, "A:% 8.*fmV        ",decimals, ch0);
			lcd_string(&lcd, str);
			lcd_cursor(&lcd, 3,0);
			sprintf(str, "%3i", encoder_count);
			lcd_string(&lcd, str);
			//printf("T1: %2d\r\n", HAL_GetTick-timer_ADS);
		}

		if (ADS_state == ADS_INIT) {
			status = setupADS1256();
			lcd_cursor(&lcd, 2,0);
			sprintf(str, "ADS1256 status: 0x%02X", status);
			lcd_string(&lcd, str);
			ADS_state = startSampling(spsi);
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
				lcd_string(&lcd, "please power cycle! ");
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

// This function is automatically called when DMA completes a transfer
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1){
	dma_complete = 1;
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
