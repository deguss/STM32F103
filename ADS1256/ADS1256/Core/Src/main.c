#include "main.h"
#include "fatfs.h"
#include "core_cm3.h"  // Adjust based on your system
#include "ads1256.h"
#include "lcd.h"
#include "stdio.h"
#include "file.h"
#include "gps.h"
#include "displays.h"

ADS_states ADS_state = ADS_INIT;
USBD_StatusTypeDef USB_state = USBD_BUSY;	//initialize with NOK at beginning
SD_states SD_state = SD_INIT;
GPS_states GPS_state = GPS_INIT;

IWDG_HandleTypeDef hiwdg;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim4;

float vdda, c0, c1, c2, c3;

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


int main(void){
	uint32_t percent, total;
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
	ITM_SendString("\n\nSWV (serial wire viewer) test. Stimulus Port 0.0 enabled\n");

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
			snprintf(str, 20, "SD:%2lu%% used of %luGB", percent, total);
			lcd_line(&lcd, str);
		}
	}

	HAL_Delay(2000);

	readWriteConfigFile("config.txt");

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


	// Enable PVD with the desired voltage threshold
	PWR_PVDTypeDef sConfigPVD;
	sConfigPVD.PVDLevel = PWR_PVDLEVEL_7; // Set PVD detection level (e.g., 2.9V)
	sConfigPVD.Mode = PWR_PVD_MODE_IT_FALLING; // Set rising edge interrupt mode

	HAL_PWR_ConfigPVD(&sConfigPVD);
	HAL_PWR_EnablePVD();

	// Enable PVD EXTI interrupt in NVIC
	HAL_NVIC_SetPriority(PVD_IRQn, 0, 0);  //highest priority interrupt to unmount SD card in case of a power loss
	HAL_NVIC_EnableIRQ(PVD_IRQn);


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
    //configure GPS to send time and date
    const char *enable_zda = "$PUBX,40,ZDA,0,1,0,0*45\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)enable_zda, strlen(enable_zda), HAL_MAX_DELAY);


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
			snprintf(str, 25, "%5dHz %1dch +-%4dmV", adcDataArray.sps, adcDataArray.channels, range[pga_index]);
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
		if (display_page == ADS && HAL_GetTick() - timer_ADS > 500){ //check if 300ms ellapsed
			if (ADS_state == ADS_RECORDING){
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
				// Format the output string in integer math.
				snprintf(str, 20, "A:%ld.%0*ldmV", ch0/1000, decimals, abs_ch0 % 1000);
				lcd_line(&lcd, str);
			}
			else if(ADS_state == ADS_INIT) {
				status = setupADS1256();
				lcd_cursor(&lcd, 2,0);
				snprintf(str, 21, "ADS1256 status: 0x%02X", status);
				lcd_line(&lcd, str);
				ADS_state = startSampling();
			}

		}	//end of display_page == ADS

		if (ADS_state == ADS_RECORDING && flagBufferFull) { //ADC receive ring buffer full
			flagBufferFull = 0;
			if (USB_state == USBD_OK && elog){
				transmitArrayOverUSB(&adcDataArray);
			}
			if (SD_state == SD_OK && GPS_state == GPS_TIME){
				arraytoFile(&adcDataArray);
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

		//ITM_SendString(GPS_rx_buf);

		if (GPS_rx_index == 0 && parse_nmea_minmea(GPS_rx_buf, &dateTimeNow, &sat_in_view)) {
			if (dateTimeNow.hours<24 && dateTimeNow.minutes<60 && dateTimeNow.seconds<60 &&
					dateTimeNow.day<32 && dateTimeNow.month<13 && dateTimeNow.year>2024 && dateTimeNow.year<3000){
				snprintf(str, 20, "%4u.%02u.%02u %2u:%02u:%02u",
						dateTimeNow.year, dateTimeNow.month, dateTimeNow.day, dateTimeNow.hours, dateTimeNow.minutes, dateTimeNow.seconds);
				GPS_state = GPS_TIME;
			}
			else {
				snprintf(str, 20, "waiting for GPS");
				GPS_state = GPS_INIT;
			}
			lcd_cursor(&lcd, 3,0);
			lcd_line(&lcd, str);
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



		updateStatesLCD();
		//HAL_IWDG_Refresh(&hiwdg);
	}

}







