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


#include "ads1256.h"


#define ADCBUFLEN 254
// struct must be of length MOD 4 = 0
typedef struct {
	uint16_t length;
	uint16_t sps;
	uint32_t time;
	uint16_t res16;
	uint8_t  res8;
	uint8_t channels;
	int32_t data[ADCBUFLEN];
} AdcDataArrayStruct;

extern volatile int32_t encoder_count;
extern volatile int dma_complete;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
void transmitArrayOverUSB(AdcDataArrayStruct *arr);

/* APP_TX_DATA_SIZE = 1024, 6 bytes header, 4 bytes per int32_t -> (1024-6)/4=254
 * packet transmit time for different sampling frequencies
 * at 30000 sps packet period = 8.50ms
 * at 15000 sps packet period = 16.80ms
 * at 7500 sps packet period = 33.60ms
 * at 3750 sps packet period = 67.20ms
 * at 2000 sps packet period = 126.40ms
 * at 1000 sps packet period = 252.80ms
 * at 500 sps packet period = 508.00ms
 * at 100 sps packet period = 2.54s 	-> buflen = 50 yields packet period of 500ms
 * at 60 sps packet period = 4.20		-> buflen = 30 yields packet period of 500ms
 * at 50 sps packet period = 5.08		-> buflen = 25 yields packet period of 500ms
 * at 30 sps packet period = 8.47 		-> buflen = 15 yields packet period of 500ms
 * at 25 sps packet period = 10.16		-> buflen = 25 yields packet period of 1s
 * at 15 sps packet period = 16.87		-> buflen = 15 yields packet period of 1s
 * at 10 sps packet period = 25.40		-> buflen = 10 yields packet period of 1s
 * at 5 sps packet period = 50.80		-> buflen = 5 yields packet period of 1s
 * at 2.5 sps packet period = 101.60	-> buflen = 5 yields packet period of 2s
 */



extern const uint8_t bufferSizes[16];
extern uint16_t flagBufferFull;
extern AdcDataArrayStruct adcDataArray;
extern uint8_t sps_index;
extern uint8_t pga_index;

extern char GPS_rx_buf[]; // Buffer to store the received string
extern uint16_t GPS_rx_index; // Index for the received_string buffer

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
void PVD_IRQHandler(void);
void USART1_IRQHandler(void);



#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_IT_H */
