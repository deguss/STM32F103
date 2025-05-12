#ifndef __FILE_H
#define __FILE_H
#include <string.h>
#include "fatfs.h"
#include "gps.h"

#define MINIM(a, b) ((a) < (b) ? (a) : (b))


extern FATFS FatFs;
extern FATFS* pfs;

FRESULT readWriteConfigFile(const char* fname);
FRESULT stat_with_lfn(const char *path, FILINFO *fno);
FRESULT writeArrayToFile(AdcDataArrayStruct *arr, dateTimeStruct* startdT);
void getSTM_UID(char *uid);
FRESULT read_config(const char* fname, const char* key, char* value, size_t value_size);
FRESULT write_config(const char* fname, const char* key, const char* value);
FRESULT write_config_line(FIL* fil, const char* key, const char* value);

void print_fresult(FRESULT res, char* buffer);
FRESULT checkSDusage(uint32_t* percent, uint32_t* total);

#endif
