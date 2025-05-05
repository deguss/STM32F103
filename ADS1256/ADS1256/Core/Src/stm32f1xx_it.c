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
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"
#include <stdbool.h>
#include "ff.h"

#define DEBOUNCE_DELAY_MS 10  // Debounce delay in milliseconds

volatile uint8_t last_a_state = 0;
volatile uint8_t last_b_state = 0;
volatile int32_t encoder_count = 0; // Use an int to hold the count
volatile uint32_t last_interrupt_time = 0; // General interrupt timing
volatile int dma_complete = 0;


extern PCD_HandleTypeDef hpcd_USB_FS;
extern DMA_HandleTypeDef hdma_adc1;

uint8_t adcData[3];

AdcDataArrayStruct adcDataArray;
uint32_t idx=0;
uint16_t flagBufferFull = 0;
uint8_t  sps_index = 6;	//default 50Hz sample rate
uint8_t pga_index = 0; //default PGA = 1
extern uint16_t Timer1, Timer2;

const uint8_t bufferSizes[16] = {
    5,
    5,
    10,
    15,
    25,
    15,
    25,
    30,
    50,
    254,
    254,
    254,
    254,
    254,
    254
};


uint8_t rx_data; // Variable to store the received byte
char GPS_rx_buf[300]; // Buffer to store the received string
uint16_t GPS_rx_index = 0; // Index for the received_string buffer

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
        		ITM_SendString("power loss detected\r\n");
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

			//store in a ring buffer
			adcDataArray.data[idx] = concatenateToInt32(adcData);

			idx++;
			if(idx >= bufferSizes[sps_index]){
				idx=0;
				adcDataArray.length = bufferSizes[sps_index];
				flagBufferFull = 1;
			}
			break;

		case GPIO_PIN_2: // PPS
			//ITM_SendString("PPS-interrupt\n");
			break;
		case GPIO_PIN_8: // BUT
			ITM_SendString("BUTTON\n");
			break;

		case GPIO_PIN_10:
		case GPIO_PIN_11:
			// Get current time in milliseconds
			uint32_t current_time = HAL_GetTick();

			// Check if this interrupt is occurring after the debounce delay
			if (current_time - last_interrupt_time < DEBOUNCE_DELAY_MS) {
				return; // Ignore if within debounce time
			}

			// Read the current state of both pins
			last_a_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10); // Read the state of ROTA
			last_b_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11); // Read the state of ROTB

			if (GPIO_Pin == GPIO_PIN_10) { // ROTA
				// Determine the direction of rotation
				if (last_b_state == GPIO_PIN_RESET) {
					encoder_count--; // Rotate left
					ITM_SendString("LEFT\n");
				} else {
					encoder_count++; // Rotate right
					ITM_SendString("RIGHT\n");
				}
			} else if (GPIO_Pin == GPIO_PIN_11) { // ROTB
				// Determine the direction of rotation
				if (last_a_state == GPIO_PIN_RESET) {
					encoder_count++; // Rotate right
					ITM_SendString("RIGHT\n");
				} else {
					encoder_count--; // Rotate left
					ITM_SendString("LEFT\n");
				}
			}

			// Update the last interrupt time
			last_interrupt_time = current_time;
			break;
/*
		case GPIO_PIN_10: // ROTA
			// Check if this interrupt is occurring after the debounce delay
			// Check debounce timing
			if (current_time - last_interrupt_time >= DEBOUNCE_DELAY_MS) {
				// Read the current state of both ROTA and ROTB
				uint8_t current_a_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
				uint8_t current_b_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);

				// Determine the direction based on the state of B
				if (last_a_state == GPIO_PIN_RESET && current_a_state == GPIO_PIN_SET) {
					// If A state changes from LOW to HIGH
					if (current_b_state == GPIO_PIN_RESET) {
						encoder_count++; // Rotate right
						ITM_SendString("RIGHT\n");
					} else {
						encoder_count--; // Rotate left
						ITM_SendString("LEFT\n");
					}
				}

				// Update last_a_state
				last_a_state = current_a_state;
				// Update the last interrupt time
				last_interrupt_time = current_time;
			}
			break;

		case GPIO_PIN_11: // ROTB
			// Check if this interrupt is occurring after the debounce delay
			// Check debounce timing
			if (current_time - last_interrupt_time >= DEBOUNCE_DELAY_MS) {
				// Read the current state of both ROTA and ROTB
				uint8_t current_a_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
				uint8_t current_b_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);

				// Determine the direction based on the state of A
				if (last_b_state == GPIO_PIN_RESET && current_b_state == GPIO_PIN_SET) {
					// If B state changes from LOW to HIGH
					if (current_a_state == GPIO_PIN_RESET) {
						encoder_count--; // Rotate left
						ITM_SendString("LEFT\n");
					} else {
						encoder_count++; // Rotate right
						ITM_SendString("RIGHT\n");
					}
				}

				// Update last_b_state
				last_b_state = current_b_state;
				// Update the last interrupt time
				last_interrupt_time = current_time;
			}
			break;
*/
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
  if (huart->Instance == USART1){
    // Process the received byte
    if (GPS_rx_index < sizeof(GPS_rx_buf) - 1){
    	GPS_rx_buf[GPS_rx_index] = rx_data;

      // Check for a newline character to indicate the end of a string
      if (rx_data == '$' || rx_data == '\r'){
    	  GPS_rx_buf[GPS_rx_index] = '\0'; // Null-terminate the string
    	  // Forward the received string to ITM_SendStr()
    	  //ITM_SendString(GPS_rx_buf);
    	  //ITM_SendString('\n');

    	  GPS_rx_index = 0; // Reset the index for the next string
      }
      else{
    	  GPS_rx_index++;
      }
    }
    else{
      // Buffer overflow, reset the buffer and start over
    	GPS_rx_index = 0;
    	ITM_SendString("GPS RX buffer overflow!\n");
    }

    if (HAL_UART_Receive_IT(&huart1, &rx_data, 1) != HAL_OK){
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
