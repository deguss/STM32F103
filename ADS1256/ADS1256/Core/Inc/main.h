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

extern IWDG_HandleTypeDef hiwdg;
void Error_Handler(void);

extern GPIO_PinState HAL_GPIO_GetOutputPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#define get_BLED() 		!HAL_GPIO_GetOutputPin(GPIOB, GPIO_PIN_12)
#define BLED(x) 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (x == 1) ? GPIO_PIN_RESET : GPIO_PIN_SET);
#define toggle_BLED() 	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);

#define is_REC_ON() 	HAL_GPIO_GetOutputPin(GPIOB, GPIO_PIN_13)
#define REC_ON() 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
#define REC_OFF() 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);

#define get_GLED() 		HAL_GPIO_GetOutputPin(GPIOB, GPIO_PIN_15)
#define GLED(x) 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, (x == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#define toggle_GLED() 	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
