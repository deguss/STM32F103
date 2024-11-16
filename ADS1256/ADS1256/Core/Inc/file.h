#ifndef __FILE_H
#define __FILE_H
#include <string.h>
#include "fatfs.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define CONFIG_FILE_NAME "config.txt"


extern FATFS FatFs;
extern FATFS* pfs;


void getSTM_UID(char *uid);
FRESULT read_config(const char* key, char* value, size_t value_size);
FRESULT write_config(const char* key, const char* value);

void print_fresult(FRESULT res, char* buffer);
FRESULT checkSDusage(uint32_t* percent, uint32_t* total);

#endif
