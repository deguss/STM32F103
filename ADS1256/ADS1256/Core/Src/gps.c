/*
 * gps.c
 *
 *  Created on: May 5, 2025
 *      Author: Daniel
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h> // Include for uint8_t
#include <stdbool.h> // Include for bool
#include "gps.h"
#include "minmea.h"
#include "displays.h"


dateTimeStruct dateTimeNow;


// Function to convert DateTime to seconds since the Unix epoch (1st January 1970)
// if dt = 0, 367,297,984 seconds or equivalent to 1981-08-26 00:00:27 UTC is returned
// Maximum timestamp the function can return: 4,294,967,295 (represents 2106-02-07 06:28:15 UTC)
uint32_t datetime_to_epoch(dateTimeStruct dt) {

	// Assumes year >= 1970
	if (dt.month < 3) {
		dt.month += 12;
		dt.year -= 1;
	}
	uint32_t timestamp = (uint32_t)(
		365L * dt.year + dt.year / 4 - dt.year / 100 + dt.year / 400 +
		(153 * (int)dt.month - 457) / 5 + (int)dt.day - 719469
	) * 86400L + (int)dt.hours * 3600 + (int)dt.minutes * 60 + (int)dt.seconds;

	// Add 27 seconds to approximate UTC with leap seconds (2025)
	timestamp += 27;
	return timestamp;
}

// Function to calculate the difference between two DateTime objects in seconds
uint32_t datetime_diff(dateTimeStruct dt1, dateTimeStruct dt2) {
    uint32_t seconds1 = datetime_to_epoch(dt1);
    uint32_t seconds2 = datetime_to_epoch(dt2);

    return seconds2 - seconds1;  // return the difference in seconds
}

// Parses a complete NMEA sentence using the minmea library
int parse_nmea_minmea(const char *nmea_sentence, dateTimeStruct *dateTime, uint8_t *sat_in_view)
{
    if (!nmea_sentence || !dateTime) {
        return MINMEA_INVALID; // null pointer
    }
    size_t len = strlen(nmea_sentence);
    // Verify checksum and structure
    if (!minmea_check(nmea_sentence, len)) {
        return MINMEA_INVALID; // invalid NMEA
    }
    enum minmea_sentence_id id = minmea_sentence_id(nmea_sentence, false); //TODO consider strict=true
    switch( id ){
		case MINMEA_SENTENCE_RMC: {
			struct minmea_sentence_rmc frame;
			if (minmea_parse_rmc(&frame, nmea_sentence) && frame.valid) {
				dateTime->hours   = frame.time.hours;
				dateTime->minutes = frame.time.minutes;
				dateTime->seconds = frame.time.seconds;
				dateTime->day     = frame.date.day;
				dateTime->month   = frame.date.month;
				dateTime->year    = frame.date.year;

				//char s[100];
				//snprintf(s,sizeof(s),"RMC lat: %ld, lon: %ld (speed: %ld)\r\n",
				//		minmea_rescale(&frame.latitude, 1000),
				//		minmea_rescale(&frame.longitude, 1000),
				//		minmea_rescale(&frame.speed, 1000));
				//ITM_SendString(s);
				return id;
			}
		} break;

    	case MINMEA_SENTENCE_ZDA: {
			struct minmea_sentence_zda frame;
			if (minmea_parse_zda(&frame, nmea_sentence)) {
				dateTime->hours   = frame.time.hours;
				dateTime->minutes = frame.time.minutes;
				dateTime->seconds = frame.time.seconds;
				dateTime->day     = frame.date.day;
				dateTime->month   = frame.date.month;
				dateTime->year    = frame.date.year;
				return id;
			}
		} break;

        case MINMEA_SENTENCE_GSV: {
            struct minmea_sentence_gsv frame;
            if (minmea_parse_gsv(&frame, nmea_sentence)) {
            	*sat_in_view = (uint8_t)frame.total_sats;
            	return id;
            }
        } break;

        case MINMEA_SENTENCE_GGA: {
            struct minmea_sentence_gga frame;
            if (minmea_parse_gga(&frame, nmea_sentence)) {
                // Evaluate fix quality for time validity
                if (frame.fix_quality > 0) {
                	char s[100];
                	//snprintf(s,sizeof(s),"GPGGA lat: %ld, lon: %ld (alt: %ld%c)\r\n",
					//	minmea_rescale(&frame.latitude, 1000), minmea_rescale(&frame.longitude, 1000),
					//	frame.altitude, frame.altitude_units);
                	snprintf(s,sizeof(s),"GPGGA GPS fix %dD lat: %i, lon: %ld (alt: %i, height: %i)\n", frame.fix_quality,
                			frame.latitude.value, frame.longitude.value, frame.altitude, frame.height);
					ITM_SendString(s);
                	// fix_quality: 1...TIME only, 2=2D, 3=3D fix

                     // The time in GGA is generally considered valid if there's a fix.
                     // Access time: frame.time
                     // printf("  Time: %d:%d:%d.%d\n", frame.time.hours, frame.time.minutes, frame.time.seconds, frame.time.microseconds);
                }
                return id;
            }
        } break;


        case MINMEA_SENTENCE_GBS: {
        	return id;
        } break;

        case MINMEA_SENTENCE_GLL: {
        	return id;
        } break;

        case MINMEA_SENTENCE_GSA: {
        	return id;
        } break;

        case MINMEA_SENTENCE_GST: {
        	return id;
        } break;

        case MINMEA_SENTENCE_VTG: {
        	return id;
        } break;

        default:          //MINMEA_INVALID = -1  or MINMEA_UNKNOWN = 0
        	return id;
    }
    return MINMEA_INVALID;

}


