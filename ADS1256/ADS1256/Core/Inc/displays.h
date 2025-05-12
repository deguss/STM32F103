/*
 * displays.h
 *
 *  Created on: May 6, 2025
 *      Author: Daniel
 */

#ifndef INC_DISPLAYS_H_
#define INC_DISPLAYS_H_

#include "lcd.h"
#include "main.h"
//#include "usb_device.h"
//#include "usbd_cdc_if.h"
#include "gps.h"

//needed for printf ITM debug output
int _write(int file, char *ptr, int len);
void ITM_Init(void);
void ITM_SendString(const char* str);
void updateStatesLCD(uint32_t percentFull, uint8_t sat_in_view);



#endif /* INC_DISPLAYS_H_ */
