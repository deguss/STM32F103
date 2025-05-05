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

uint8_t hours, minutes, seconds;

int parse_gprmc_datetime(const char *nmea_sentence, char *date_str_out, uint8_t *hours_out, uint8_t *minutes_out, uint8_t *seconds_out, GPS_states *fix_status_out) {
  // Error handling: Check for null pointers.
  if (!nmea_sentence || !date_str_out || !hours_out || !minutes_out || !seconds_out) {
    return -1; // Indicate an error: NULL pointer passed.
  }

  //  Crucial: Check if the input starts with "GPRMC,".
  if (strncmp(nmea_sentence, "GPRMC,", 6) != 0) { // Adjusted length to 6
    return -2; // Indicate invalid NMEA sentence format (doesn't start with GPRMC,)
  }

  // Buffer for extracting time and date data.  Crucial for safety!
  char time_str[12]; // Increased size to accommodate potential fractional seconds
  char date_str[7];  // Date is DDMMYY
  char status_char;

  // Use sscanf to parse the relevant fields.
  // The format string needs to accommodate the optional fields between time and date.
  // We'll use %*[^,] to skip fields we don't need for date/time extraction.
  // %c will read the status character.
  // %6[^,] will read the date field.

  // Removed the "$" from the sscanf format string
  int result = sscanf(nmea_sentence, "GPRMC,%11[^,],%c,%*[^,],%*[^,],%*[^,],%*[^,],%*[^,],%*[^,],%6[^,],%*[^\n]",
                      time_str, &status_char, date_str);

  // Check the result of sscanf. We expect to successfully read time, status, and date.
  // However, the date field can sometimes be missing in 'V' status sentences.
  // Let's handle the case where only time and status are parsed.
  if (result < 2) {
      return -3; // Indicate a general parsing error (couldn't even get time and status).
  }

  // Determine the fix status based on the status character
  if (status_char == 'A') {
      fix_status_out = GPS_FIX;
  } else if (status_char == 'V') {
      fix_status_out = GPS_TIME;
  } else {
      fix_status_out = GPS_ERROR; // Handle other potential status characters
  }

  // Handle the case where the date field was not successfully parsed (result == 2)
  if (result == 2) {
      // Date field was not parsed. This is common for 'V' status sentences.
      // We can still extract time, but date will be empty.
      date_str_out[0] = '\0'; // Set date string to empty
      // Continue to parse time
  } else if (result == 3) {
      // Time, status, and date were all parsed successfully.
      // Copy the date string to the output buffer.
      strcpy(date_str_out, date_str);
  } else {
      // Unexpected sscanf result
      return -4; // Indicate an unexpected parsing outcome.
  }


  // Extract time components (using strtol is safer than atoi)
  char *decimal_point = strchr(time_str, '.');
  if (decimal_point) {
    *decimal_point = '\0'; // Null-terminate at the decimal point
  }

  long time_long = strtol(time_str, NULL, 10);
  if (time_long < 0 || time_long > 235959) {
      return -5; // Error in extracted time component (value out of range).
  }

  *hours_out = (uint8_t)(time_long / 10000);
  *minutes_out = (uint8_t)((time_long % 10000) / 100);
  *seconds_out = (uint8_t)(time_long % 100);

  return 0; // Success
}
