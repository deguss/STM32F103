/*
 * gps.h
 *
 *  Created on: May 5, 2025
 *      Author: Daniel
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_
#include "displays.h"

typedef struct datetime {
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
    uint8_t day;
    uint8_t month;
    uint16_t year;
} dateTimeStruct;

extern dateTimeStruct dateTimeNow;


uint32_t datetime_to_epoch(dateTimeStruct dt);
uint32_t datetime_diff(dateTimeStruct dt1, dateTimeStruct dt2) ;

int parse_nmea_minmea(const char *nmea_sentence, dateTimeStruct *dateTime, uint8_t *sat_in_view);



#endif /* INC_GPS_H_ */
