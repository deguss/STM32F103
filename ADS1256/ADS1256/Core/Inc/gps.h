/*
 * gps.h
 *
 *  Created on: May 5, 2025
 *      Author: Daniel
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

extern uint8_t hours, minutes, seconds;

typedef enum {
	GPS_INIT,
	GPS_TIME,
	GPS_FIX,
	GPS_ERROR } GPS_states;


int parse_gprmc_datetime(const char *nmea_sentence, char *date_str_out, uint8_t *hours_out, uint8_t *minutes_out, uint8_t *seconds_out, GPS_states *fix_status_out);

#endif /* INC_GPS_H_ */
