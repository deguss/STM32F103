#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "ads1256.h"


IWDG_HandleTypeDef hiwdg;
SPI_HandleTypeDef hspi1;



#define USB_TX_BUF_SIZE 256
uint8_t usbTx[USB_TX_BUF_SIZE];

void transmitArrayOverUSB(AdcDataArrayStruct *arr);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_IWDG_Init(void);


static uint8_t startSampling(void){
	//configure single ended: AIN0 referred to AINCOM
	// test if communication to ADC works (read by register)
	if (setChannel(0,-1)){	//if not succeeded
		return 1;
	}
	setGain(SPS_50, PGA1);	//continuous conversation starts here

	adcDataArray.length = ADCBUFLEN;
	adcDataArray.sps = 50;
	adcDataArray.INP = 0;
	adcDataArray.INM = -1;

	HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 7);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn); //enable DRDY interrupt
	return 0; //for success
}


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


//-----------------------------------------------------------------------------
int main(void){
	static USBD_StatusTypeDef usb_status=USBD_FAIL;	//initialize with NOK at beginning
	static uint32_t tick_1s;

	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	BLED(1) 	// signaling the start of the INIT

	MX_IWDG_Init();
	MX_USB_DEVICE_Init();


	MX_SPI1_Init();
	uint8_t status;
	status = setupADS1256();	//should be 0x30 for this particular piece of AD1256 (ID)
	sprintf((char *)usbTx, "ADS1256 data logger (status:0x%02X) on the 72MHz SMT32F108\r\n",status);

	BLED(0)	//end INIT

	tick_1s = HAL_GetTick();	//start 1s timer

	while (1){


		if (usb_status ==  USBD_OK){
			if (!is_REC_ON()) {
				if (startSampling() == 0){	//if succeeded starting
					REC_ON()
				}
			}
			if (flagBufferFull){  //ADC receive ring buffer full
				flagBufferFull = 0;
				transmitArrayOverUSB(&adcDataArray);
				toggle_GLED()
			}
			keepBlueLEDFlashing();
		}
		else { //if USB status not OK
			if (!is_REC_ON()) {	//and if it was not connected before
				if (HAL_GetTick() - tick_1s > 1000){ //check if 1s ellapsed
					usb_status = CDC_Transmit_FS(usbTx, strlen((char *)usbTx));	//transmit msg
					tick_1s = HAL_GetTick();	//update timer with present value of time
				}
			}
			else {	//but it was recording and lost somehow USB connection
				REC_OFF()
				HAL_NVIC_DisableIRQ(EXTI0_IRQn); //disable DRDY interrupt
				stopSampling();
			}
			BLED(0)
		}

		HAL_IWDG_Refresh(&hiwdg);
	}

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
    size_t totalSize = sizeof(AdcDataArrayStruct);
    size_t ofs;

    // Buffer to store the data to be sent
    uint8_t buffer[totalSize];

    memcpy(buffer, &arr->length,sizeof(uint16_t));
    ofs = sizeof(uint16_t);

    memcpy(buffer+ofs, &arr->sps, 	sizeof(uint16_t));
    ofs += sizeof(uint16_t);

    memcpy(buffer+ofs, &arr->INP, 	sizeof(int8_t));
    ofs += sizeof(int8_t);

    memcpy(buffer+ofs, &arr->INM, 	sizeof(int8_t));
    ofs += sizeof(int8_t);

    memcpy(buffer+ofs, arr->data, arr->length * sizeof(int32_t));

    // Transmit the data over USB CDC
    CDC_Transmit_FS(buffer, totalSize);
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void) {

	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK){
		Error_Handler();
	}

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void){

  GPIO_InitTypeDef GPIO_InitStruct = {0};


  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();



   /*Configure GPIOA pin 4 NSS (chip select) */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIOA pin 6 MISO */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : GPIO_EXTI0_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_0;				//INT0: DRDY
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1); //B12 blue LED (1: off, 0: on)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0); //B13 red LED
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0); //B15 green LED
	/*Configure GPIO pins : PB13 and PB15 */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

GPIO_PinState HAL_GPIO_GetOutputPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){

  /* Check the parameters */
  assert_param(IS_GPIO_PIN(GPIO_Pin));

  /* get current Output Data Register value */
  uint32_t odr = GPIOx->ODR;

  return (GPIO_PinState)((odr & GPIO_Pin) != 0);
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
