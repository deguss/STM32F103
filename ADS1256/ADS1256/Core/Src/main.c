#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "ads1256.h"
#include "lcd.h"
#include "stdio.h"




IWDG_HandleTypeDef hiwdg;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

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
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_IWDG_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);


static int dma_complete = 0;

static ADS_states startSampling(void){
	//configure single ended: AIN0 referred to AINCOM
	// test if communication to ADC works (read by register)
	if (setChannel(0,1)){	//if not succeeded
		return ADS_ERROR;
	}
	sps_index = 6;
	pga_index = 0;
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
	static char str[21];

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

	static uint32_t timer_USB, timer_intADC, timer_ADS, timer_mainloop;
	static char str[21];
	uint16_t intADCraw[5];
	static float vrefint = 1200;
	//float vdda, c0, c1, c2, c3;
	//float ch0, ch1, ch2, ch3;
	enum display_pages {ADS, INTADC} display_page = ADS;


	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();

	HAL_Delay(200);
	printf("SWV (serial wire viewer) test. Stimulus Port 0.0 enabled\r\n");

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
	HAL_Delay(3000);


	MX_USB_DEVICE_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK){
		lcd_clear(&lcd);
		lcd_cursor(&lcd, 1, 0);
		lcd_string(&lcd, "internal ADC calibr. failed!");
		Error_Handler();
	}

	lcd_clear(&lcd);
	if ( HAL_ADC_Start_DMA(&hadc1, (uint32_t*)(intADCraw), hadc1.Init.NbrOfConversion) ){
		lcd_clear(&lcd);
		lcd_string(&lcd, "ADC DMA setup failed!");
		Error_Handler();
	}


	MX_SPI1_Init();	//interface to ADS1256
	//MX_SPI2_Init();	//interface to SD-card
	//MX_USART1_UART_Init();
	uint8_t status;
	status = setupADS1256();	//should be 0x30 for this particular piece of AD1256 (ID)


	timer_USB = HAL_GetTick();	//start timers
	timer_ADS = HAL_GetTick();
	timer_intADC = HAL_GetTick();

	//MX_IWDG_Init();
	//__HAL_DBGMCU_FREEZE_IWDG();
	lcd_clear(&lcd);
	while (1){
		timer_mainloop = HAL_GetTick();

		if (usb_received){
			printf("USB_received triggered\n");
			lcd_cursor(&lcd, 3,0);
			snprintf(str, usb_received+1, "%s", UserRxBufferFS);
			lcd_string(&lcd, str);
			usb_received = 0;
		}

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
			//printf("T1: %2d\r\n", HAL_GetTick-timer_ADS);
		}

		if (ADS_state == ADS_INIT) {
			status = setupADS1256();
			lcd_cursor(&lcd, 2,0);
			sprintf(str, "ADS1256 status: 0x%02X", status);
			lcd_string(&lcd, str);
			ADS_state = startSampling();
		}
		if (ADS_state == ADS_RECORDING && flagBufferFull) { //ADC receive ring buffer full
			flagBufferFull = 0;
			if (USB_state == USBD_OK){
				//transmitArrayOverUSB(&adcDataArray);
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


//needed for prinf ITM debug output
int _write(int file, char *ptr, int len){
	for (int i=0; i<len; i++){
		ITM_SendChar(*ptr++);
	}
	return len;
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }

}


/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);


}


/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void){
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void){
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

// SPI1 dedicated to external ADC: ADS1256
static void MX_SPI1_Init(void) {

	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; //1.12MHz
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK){
		Error_Handler();
	}

}

// SPI2 SD-card
static void MX_SPI2_Init(void) {

	hspi1.Instance = SPI2;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; //1.12MHz
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK){
		Error_Handler();
	}

}

static void MX_GPIO_Init(void){

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();


   /*Configure GPIOB pin 4,5,8,9 for LCD D4,D5,D6,D7 */
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIOA pin 4 for LCD_RS */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIOC pin 13 for LCD_E */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


	/*Configure GPIO pin : EXTI0: DRDY, EXTI2: PPS, EXTI10: ROTA, EXTI11: ROTB */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    /*Configure GPIOB pin 14 MISO (SPI2)  -> HAL_SPI_MspInit() */

	/*Configure GPIOA pin 0..3 analog input -> void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc) */


}

GPIO_PinState HAL_GPIO_GetOutputPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){

  /* Check the parameters */
  assert_param(IS_GPIO_PIN(GPIO_Pin));

  /* get current Output Data Register value */
  uint32_t odr = GPIOx->ODR;

  return (GPIO_PinState)((odr & GPIO_Pin) != 0);
}



/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
