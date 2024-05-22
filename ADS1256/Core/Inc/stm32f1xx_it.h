/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
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
#ifndef __STM32F1xx_IT_H
#define __STM32F1xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ads1256.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern SPI_HandleTypeDef hspi1;

#define ADCBUFLEN 254
/* APP_TX_DATA_SIZE = 1024, 6 bytes header, 4 bytes per int32_t -> (1024-6)/4=254
 * packet transmit time for different sampling frequencies
 * at 30000 sps packet period = 8.50ms
 * at 15000 sps packet period = 16.80ms
 * at 7500 sps packet period = 33.60ms
 * at 3750 sps packet period = 67.20ms
 * at 2000 sps packet period = 126.40ms
 * at 1000 sps packet period = 252.80ms
 * at 500 sps packet period = 508.00ms
 */

typedef struct {
    uint16_t length;
    uint16_t sps;
    int8_t INP;
    int8_t INM;
    int32_t data[ADCBUFLEN];
} AdcDataArrayStruct;

extern uint16_t flagBufferFull;
extern AdcDataArrayStruct adcDataArray;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void USB_LP_CAN1_RX0_IRQHandler(void);

void EXTI0_IRQHandler(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_IT_H */
