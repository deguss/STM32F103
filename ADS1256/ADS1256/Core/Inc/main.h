/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"
//#include "usbd_def.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"




typedef enum {
	ADS_INIT,
	ADS_READY,
	ADS_RECORDING,
	ADS_ERROR}  ADS_states;
typedef enum {
	SD_INIT,
	SD_OK,
	SD_FSERROR } SD_states;
typedef enum {
	GPS_INIT,
	GPS_TIME,
	GPS_FIX,
	GPS_ERROR } GPS_states;


extern IWDG_HandleTypeDef hiwdg;
extern ADS_states ADS_state;
extern SD_states SD_state;
extern GPS_states GPS_state;
extern USBD_StatusTypeDef USB_state;
extern uint8_t config_channels;


void Error_Handler(void);

extern GPIO_PinState HAL_GPIO_GetOutputPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
extern void ITM_SendString(const char* str);

#define get_PA15() 		HAL_GPIO_GetOutputPin(GPIOA, GPIO_PIN_15)
#define PA15(x) 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, (x == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#define toggle_PA15() 	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);


void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_SPI1_Init(SPI_HandleTypeDef* hspi1);
void MX_SPI2_Init(SPI_HandleTypeDef* hspi2);
void MX_IWDG_Init(void);
void MX_I2C1_Init(I2C_HandleTypeDef* hi2c1);
void MX_USART1_UART_Init(UART_HandleTypeDef* huart1);
void MX_DMA_Init(void);
void MX_ADC1_Init(ADC_HandleTypeDef* hadc1);
void MX_TIM4_Init(TIM_HandleTypeDef* htim4);



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
