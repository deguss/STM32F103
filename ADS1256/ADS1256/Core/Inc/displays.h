/*
 * displays.h
 *
 *  Created on: May 6, 2025
 *      Author: Daniel
 */

#ifndef INC_DISPLAYS_H_
#define INC_DISPLAYS_H_

#include <stdint.h>
#include "stm32f1xx.h"
#include "core_cm3.h"

//needed for printf ITM debug output
int _write(int file, char *ptr, int len);
void ITM_Init(void);
void update_display_config(void);
void updateStatesLCD(uint32_t percentFull, uint8_t sat_in_view);


#ifdef DEBUG
#define ITM_SendString(str)    do { \
                                const char* s = str; \
                                while (*s) { \
                                    ITM_SendChar(*s++); \
                                } \
                            } while(0)
#else
#define ITM_SendString(str)    ((void)0)  // No-op in production
#endif



#endif /* INC_DISPLAYS_H_ */
