#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "ads1256.h"
#include "lcd.h"


IWDG_HandleTypeDef hiwdg;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;




void transmitArrayOverUSB(AdcDataArrayStruct *arr);
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

static uint8_t startSampling(void){
	//configure single ended: AIN0 referred to AINCOM
	// test if communication to ADC works (read by register)
	if (setChannel(0,-1)){	//if not succeeded
		return 1;
	}
	sps_index = 8;
	setGain(sps_const[sps_index], PGA1);	//continuous conversation starts here

	adcDataArray.sps = sps[sps_index];
	adcDataArray.length = bufferSizes[sps_index];
	adcDataArray.INP = 0;
	adcDataArray.INM = -1;

	HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 7);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn); //enable DRDY interrupt
	return 0; //for success
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

//-----------------------------------------------------------------------------
int main(void){
	static USBD_StatusTypeDef usb_status=USBD_FAIL;	//initialize with NOK at beginning
	static uint32_t tick_1s;
	static char str[20];
	static uint8_t ret;
	uint16_t intADCraw[5];
	static float vrefint = 1200;
	float vdda, c0, c1, c2, c3;
	static enum {
		STATE_INIT,
		STATE_RECORDING,
		STATE_ERROR
	} state = STATE_INIT;


	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();

	Lcd_PortType ports[] = {GPIOB, GPIOB, GPIOB, GPIOB};
	Lcd_PinType pins[] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_8, GPIO_PIN_9};
	Lcd_HandleTypeDef lcd = lcd_create(ports, pins, GPIOA, GPIO_PIN_4, GPIOC, GPIO_PIN_13, LCD_4_BIT_MODE);
	lcd_clear(&lcd);
	lcd_string(&lcd, "24-bit data logger  ");
	lcd_cursor(&lcd, 1, 0);
	lcd_string(&lcd, "Piri Daniel 2024");
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


	tick_1s = HAL_GetTick();	//start 1s timer

	MX_IWDG_Init();
	lcd_clear(&lcd);
	while (1){

		if(dma_complete){
			dma_complete=0;
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)(intADCraw), hadc1.Init.NbrOfConversion);
			lcd_cursor(&lcd, 1,0);
			vdda = vrefint / (float)(intADCraw[4]);	//calculate actual VDD based on VREFINT measurement
			sprintf(str, "VDDA = %04imV ", (int)(vdda*4095.0));
			lcd_string(&lcd, str);
			if (vdda > 1.0 || vdda < 0.7){
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
			HAL_Delay(300);

		}
		if (usb_status ==  USBD_OK){
			if (state == STATE_INIT) {
				ret = startSampling();
				if (ret == 0){	//if succeeded starting
					state = STATE_RECORDING;
					lcd_cursor(&lcd, 0, 0);
					lcd_string(&lcd, "[REC]");
					lcd_cursor(&lcd, 0, 15);
					lcd_string(&lcd, "[USB]");
				}
				else {
					state = STATE_ERROR;
					lcd_cursor(&lcd, 0, 0);
					lcd_string(&lcd, "[");
					lcd_int(&lcd, ret);
					lcd_cursor(&lcd, 0, 4);
					lcd_string(&lcd, "]");

				}
			}
			if (flagBufferFull){  //ADC receive ring buffer full
				flagBufferFull = 0;
				transmitArrayOverUSB(&adcDataArray);
			}
		}
		else { //if USB status not OK
			if (state == STATE_INIT) {	//and if it was not connected before
				if (HAL_GetTick() - tick_1s > 1000){ //check if 1s ellapsed
					usb_status = CDC_Transmit_FS(&status, 1);	//transmit msg
					tick_1s = HAL_GetTick();	//update timer with present value of time
					lcd_cursor(&lcd, 0, 15);
					lcd_string(&lcd, "[");
					lcd_int(&lcd, usb_status);
					lcd_cursor(&lcd, 0, 19);
					lcd_string(&lcd, "]");
				}
			}
			else {	//but it was recording and lost somehow USB connection
				lcd_cursor(&lcd, 0, 0);
				lcd_string(&lcd, "[   ]");
				lcd_cursor(&lcd, 0, 15);
				lcd_string(&lcd, "[   ]");
				HAL_NVIC_DisableIRQ(EXTI0_IRQn); //disable DRDY interrupt
				stopSampling();
			}
		}

		HAL_IWDG_Refresh(&hiwdg);
	}

}

// This function is automatically called when DMA completes a transfer
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1){
	dma_complete = 1;
}


// Function to transmit variable length int32_t array over USB CDC
void transmitArrayOverUSB(AdcDataArrayStruct *arr){
/*typedef struct {
    uint16_t length;
    uint16_t sps;
    int8_t INP;
    int8_t INM;
    int32_t data[ADCBUFLEN];
} AdcDataArrayStruct;*/

    // Calculate the total size of the data to be sent
    size_t totalSize = 2*sizeof(uint16_t) + 2*sizeof(int8_t) + arr->length*sizeof(int32_t);

    // Transmit the data over USB CDC
    CDC_Transmit_FS((uint8_t *)arr, totalSize);
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
