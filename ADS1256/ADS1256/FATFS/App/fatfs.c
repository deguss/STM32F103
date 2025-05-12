/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
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
#include "fatfs.h"
#include "gps.h"

uint8_t retUSER;    /* Return value for USER */
char USERPath[4];   /* USER logical drive path */
FATFS USERFatFS;    /* File system object for USER logical drive */
FIL USERFile;       /* File object for USER */

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
void MX_FATFS_Init(void){
  /*## FatFS: Link the USER driver ###########################*/
  retUSER = FATFS_LinkDriver(&USER_Driver, USERPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */

DWORD get_fattime(void)
{
    // Check for sanity — if GPS hasn't provided a valid year yet, return a safe default
    if (dateTimeNow.year < 1980) {
        return ((DWORD)(2025 - 1980) << 25) | ((DWORD)1 << 21) | ((DWORD)1 << 16);
    }

    return  ((DWORD)(dateTimeNow.year - 1980) << 25)
          | ((DWORD)dateTimeNow.month << 21)
          | ((DWORD)dateTimeNow.day << 16)
          | ((DWORD)dateTimeNow.hours << 11)
          | ((DWORD)dateTimeNow.minutes << 5)
          | ((DWORD)(dateTimeNow.seconds / 2));
}
