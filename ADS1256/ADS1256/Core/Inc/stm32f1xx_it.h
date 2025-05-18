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


#define ADCBUFLEN 125
// struct must be of length MOD 4 = 0
typedef struct {
	uint16_t length;
	uint16_t sps;
	uint32_t epochtime;
	uint8_t gain;
	uint8_t channel;
	uint16_t res16;
	int32_t data[ADCBUFLEN];
} AdcDataArrayStruct;

extern volatile int32_t encoder_count;
extern volatile int dma_complete;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

/* APP_TX_DATA_SIZE = 512, 12 bytes header, 4 bytes per int32_t -> (512-12)/4=125
 * packet transmit time for different sampling frequencies
 * at 30000 sps packet period = 4.17ms
 * at 15000 sps packet period = 8.33ms
 * at 7500 sps packet period = 16.67ms
 * at 3750 sps packet period = 33.33ms
 * at 2000 sps packet period = 62.50ms
 * at 1000 sps packet period = 125.00ms
 * at 500 sps packet period = 250.00ms
 * at 100 sps packet period = 1.25s 	-> buflen = 50 yields packet period of 500ms
 * at 60 sps packet period = 2.08s		-> buflen = 30 yields packet period of 500ms
 * at 50 sps packet period = 2.50s		-> buflen = 25 yields packet period of 500ms
 * at 30 sps packet period = 4.17s 		-> buflen = 15 yields packet period of 500ms
 * at 25 sps packet period = 5.00s		-> buflen = 25 yields packet period of 1s
 * at 15 sps packet period = 8.33s		-> buflen = 15 yields packet period of 1s
 * at 10 sps packet period = 12.50s		-> buflen = 10 yields packet period of 1s
 * at 5 sps packet period = 25.00s		-> buflen = 5 yields packet period of 1s
 * at 2.5 sps packet period = 50.00s	-> buflen = 5 yields packet period of 2s
 */



extern const uint8_t bufferSizes[16];
extern uint16_t flagBufferFull;
extern AdcDataArrayStruct adcDataArray[2];
extern uint8_t arrW_idx;	//index of adcDataArray for writing
extern uint8_t arrR_idx;	//index of adcDataArray for reading
extern uint8_t sps_index;
extern uint8_t pga_index;
extern volatile int8_t sig_enc;
extern volatile int8_t sig_btn;

extern char GPS_rx_buf[]; // Buffer to store the received string
extern uint16_t GPS_rx_index; // Index for the received_string buffer

void transmitArrayOverUSB(AdcDataArrayStruct *arr);

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
