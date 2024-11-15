#ifndef __FILE_H
#define __FILE_H

#include "fatfs.h"
#define MIN(a, b) ((a) < (b) ? (a) : (b))

extern FATFS FatFs;
extern FATFS* pfs;
extern FIL fil;
extern DWORD fre_clust;
extern uint32_t totalSpace, freeSpace, usedPercent;


void print_fresult(FRESULT res);
FRESULT mountSD(void);
FRESULT checkSDusage(uint32_t* percent, uint32_t* total);

#endif
