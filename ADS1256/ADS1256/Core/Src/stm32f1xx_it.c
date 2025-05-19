/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_it.h"
#include <stdbool.h>
#include "main.h"
#include "stm32f1xx_hal.h"
#include "ff.h"
#include "displays.h"

#define DEBOUNCE_DELAY_MS 20  // Debounce delay in milliseconds
//#define TESTDATA 1

uint8_t last_a_state = 0;
uint8_t last_b_state = 0;
volatile int8_t sig_enc = 0;
volatile int8_t sig_btn = 0;
uint32_t debounce1, debounce2;
volatile int dma_complete = 0;


extern PCD_HandleTypeDef hpcd_USB_FS;
extern DMA_HandleTypeDef hdma_adc1;

uint8_t adcData[3];

AdcDataArrayStruct adcDataArray[2];
uint8_t arrW_idx=0;	//index of adcDataArray for writing
uint8_t arrR_idx=1;	//index of adcDataArray for reading
uint32_t idx=0;		//index within adcDataArray.data
uint16_t flagBufferFull = 0;
uint8_t  sps_index = 6;	//default 50Hz sample rate
uint8_t pga_index = 0; //default PGA = 1
extern uint16_t Timer1, Timer2;

const uint8_t bufferSizes[16] = {
    5,	//2.5Hz
    5,	//5Hz
    10,	//10Hz
    15,	//15Hz
    25,	//25Hz
    15,	//30Hz
    25,	//50Hz
    30,	//60Hz
    50,	//100Hz
    125,//500Hz
    125,//1kHz
    125,//2kHz
    125,//3.75kHz
    125,//7.5kHz
    125,//15kHz
	125 //30kHz
};


uint8_t rx_data; // Variable to store the received byte
char GPS_rx_buf[200]; // Buffer to store the received string
uint16_t GPS_rx_index = 0; // Index for the received_string buffer

// Function to transmit variable length int32_t array over USB CDC
void transmitArrayOverUSB(AdcDataArrayStruct *arr){
/*typedef struct {
	uint16_t length;
	uint16_t sps;
	uint32_t epochtime;
	uint8_t gain;
	uint8_t channel;
	uint16_t res16;
	int32_t data[ADCBUFLEN];
} AdcDataArrayStruct;*/

    // Calculate the total size of the data to be sent
    size_t totalSize = 2+2+4+1+1+2 + arr->length*sizeof(int32_t);

    // Transmit the data over USB CDC
    CDC_Transmit_FS((uint8_t *)arr, totalSize);
}


void PVD_IRQHandler(void){
    HAL_PWR_PVD_IRQHandler();
}


void HAL_PWR_PVDCallback(void)
{
    // Check for the PVD interrupt flag
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO)){ // Check the PVD output flag
        // Attempt to unmount the SD card to ensure data integrity
        if (f_mount(NULL, "", 1) == FR_OK){  // Unmounting the filesystem
        	while(1){
        		HAL_Delay(1);
        	}
        }
        // Clear the PVD interrupt flag
        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_PVDO);
    }
}


void EXTI0_IRQHandler(void){
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
void EXTI2_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

void EXTI9_5_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
}

void EXTI15_10_IRQHandler(void) {
    // Handle EXTI lines 10, 11
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10) != RESET) {
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10); // Call the handler for EXTI pin 10
    }

    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11) != RESET) {
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11); // Call the handler for EXTI pin 11
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    // Get current time in milliseconds
    //uint32_t current_time = HAL_GetTick();

	switch (GPIO_Pin){
		case (GPIO_PIN_0): // DATA READY INTTERUPT FROM ADS1256
			//will be called on a falling edge of DRDY
			HAL_SPI_Receive(&hspi1, adcData, 3, HAL_MAX_DELAY); // Receive 3 bytes of data

			//store in first or second ring buffer
			adcDataArray[arrW_idx].data[idx] = concatenateToInt32(adcData);
#if defined(DEBUG) && defined(TESTDATA)
			adcDataArray[arrW_idx].data[idx] = arrW_idx+1;
#warning "TESTDATA ACTIVE!!!"
#endif
			idx++;
			if(idx >= bufferSizes[sps_index]){
				idx=0;
				adcDataArray[arrW_idx].length = bufferSizes[sps_index];
				flagBufferFull = 1;
				//switch over to next ring buffer
				arrR_idx=arrW_idx;		//read lags 1 behind write
				arrW_idx++;
				if (arrW_idx>1){
					arrW_idx = 0;
				}
			}
			break;


		case GPIO_PIN_2: // PPS
			//ITM_SendString("PPS-interrupt\n");
			break;



		case GPIO_PIN_8: // BUTTON
			// Check if this interrupt is occurring after the debounce delay
			if (HAL_GetTick() - debounce1 < DEBOUNCE_DELAY_MS) {
				return; // Ignore if within debounce time
			}
			debounce1 = HAL_GetTick();
			sig_btn = 1; //
			ITM_SendString("BUTTON\n");
			break;



		case GPIO_PIN_10:
		case GPIO_PIN_11:
			// Check if this interrupt is occurring after the debounce delay
			if (HAL_GetTick() - debounce2 < DEBOUNCE_DELAY_MS) {
				return; // Ignore if within debounce time
			}
			debounce2 = HAL_GetTick();

			// Read the current state of both pins
			last_a_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10); // Read the state of ROTA
			last_b_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11); // Read the state of ROTB

			if (GPIO_Pin == GPIO_PIN_10) { // ROTA
				// Determine the direction of rotation
				if (last_b_state == GPIO_PIN_RESET) {
					sig_enc = -1; // Rotate left
					ITM_SendString("LEFT\n");
				} else {
					sig_enc = 1; // Rotate right
					ITM_SendString("RIGHT\n");
				}
			} else if (GPIO_Pin == GPIO_PIN_11) { // ROTB
				// Determine the direction of rotation
				if (last_a_state == GPIO_PIN_RESET) {
					sig_enc = 1; // Rotate right
					ITM_SendString("RIGHT\n");
				} else {
					sig_enc = -1; // Rotate left
					ITM_SendString("LEFT\n");
				}
			}


			break;

		// Optionally, handle default case
		default:
			while(1);
			// Handle unknown GPIO_Pin values
			break;
	}


}


// This function is automatically called when DMA completes a transfer
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1){
	dma_complete = 1;
}


void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if (huart->Instance == USART1) {
      // Process incoming NMEA characters into GPS_rx_buf
      if (rx_data == '$') {
          // Start of new NMEA sentence
          GPS_rx_index = 0;
          GPS_rx_buf[GPS_rx_index++] = rx_data;
      }
      else if (rx_data == '\n') {
          // End of sentence
          GPS_rx_buf[GPS_rx_index] = '\0';
          // Sentence ready: parse or forward
          //ITM_SendString(GPS_rx_buf);
          GPS_rx_index = 0;
      }
      else {
          // Middle of sentence
          if (GPS_rx_index < sizeof(GPS_rx_buf) - 1) {
              GPS_rx_buf[GPS_rx_index++] = rx_data;
          } else {
              // Buffer overflow: reset buffer
              GPS_rx_index = 0;
          }
      }

      // Restart UART receive interrupt for next byte
      if (HAL_UART_Receive_IT(&huart1, &rx_data, 1) != HAL_OK) {
          ITM_SendString("UART Receive IT returned NOK ");
      }
  }
}

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	if(Timer1 > 0)
		Timer1--;
	if(Timer2 > 0)
		Timer2--;
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  * NONO this is called when transmitting!!!
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	/* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

	/* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
	HAL_PCD_IRQHandler(&hpcd_USB_FS);
	/* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

	/* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
