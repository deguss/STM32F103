#include "file.h"

FATFS FatFs;
FATFS* pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;



FRESULT  mountSD(void){

	// mount the FS
	fres = f_mount(&FatFs, "", 1);
	// will return FR_NOT_READY: The physical drive cannot work   if no SD card inserted
	// returns FR_NO_FILESYSTEM: There is no valid PRIMARY formatted FAT32 partition on the SD card
	if (fres != FR_OK) {
		print_fresult(fres);
		return fres;
	}
	return fres;
}

FRESULT checkSDusage(uint32_t* percent, uint32_t* total){
	float totalSpace, freeSpace, usedPercent;

	// Check free space
	fres = f_getfree("", &fre_clust, &pfs);
	if (fres != FR_OK){
		print_fresult(fres);
		return fres;
	}

	totalSpace = (float)((pfs->n_fatent - 2) * pfs->csize * 0.5);
	freeSpace = (float)(fre_clust * pfs->csize * 0.5);
	usedPercent = 100.0 - (freeSpace*100 / totalSpace);
#ifdef DEBUG
	printf("total: %9.0fkB\n", totalSpace);
	printf("free: %10.0fkB\n", freeSpace);
	printf("%2.2f%% free\n", freeSpace*100 / totalSpace);
	printf("%2.2f%% used\n", usedPercent);
#endif
	*total = MIN(((uint32_t)(totalSpace / 1000 / 1000)), 100);
	*percent = MIN(((uint32_t)(usedPercent)), 99);
	return fres;

}


void print_fresult(FRESULT res) {
#ifdef DEBUG
    switch (res) {
        case FR_OK:
            printf("FR_OK: Succeeded\n");
            break;
        case FR_DISK_ERR:
            printf("FR_DISK_ERR: A hard error occurred in the low level disk I/O layer\n");
            break;
        case FR_INT_ERR:
            printf("FR_INT_ERR: Assertion failed\n");
            break;
        case FR_NOT_READY:
            printf("FR_NOT_READY: The physical drive cannot work\n");
            break;
        case FR_NO_FILE:
            printf("FR_NO_FILE: Could not find the file\n");
            break;
        case FR_NO_PATH:
            printf("FR_NO_PATH: Could not find the path\n");
            break;
        case FR_INVALID_NAME:
            printf("FR_INVALID_NAME: The path name format is invalid\n");
            break;
        case FR_DENIED:
            printf("FR_DENIED: Access denied due to prohibited access or directory full\n");
            break;
        case FR_EXIST:
            printf("FR_EXIST: Access denied due to prohibited access (file already exists)\n");
            break;
        case FR_INVALID_OBJECT:
            printf("FR_INVALID_OBJECT: The file/directory object is invalid\n");
            break;
        case FR_WRITE_PROTECTED:
            printf("FR_WRITE_PROTECTED: The physical drive is write protected\n");
            break;
        case FR_INVALID_DRIVE:
            printf("FR_INVALID_DRIVE: The logical drive number is invalid\n");
            break;
        case FR_NOT_ENABLED:
            printf("FR_NOT_ENABLED: The volume has no work area\n");
            break;
        case FR_NO_FILESYSTEM:
            printf("FR_NO_FILESYSTEM: There is no valid PRIMARY filesystem\n");
            break;
        case FR_MKFS_ABORTED:
            printf("FR_MKFS_ABORTED: The f_mkfs() aborted due to any parameter error\n");
            break;
        case FR_TIMEOUT:
            printf("FR_TIMEOUT: Could not get a grant to access the volume within defined period\n");
            break;
        case FR_LOCKED:
            printf("FR_LOCKED: The operation is rejected according to the file sharing policy\n");
            break;
        case FR_NOT_ENOUGH_CORE:
            printf("FR_NOT_ENOUGH_CORE: LFN working buffer could not be allocated\n");
            break;
        case FR_TOO_MANY_OPEN_FILES:
            printf("FR_TOO_MANY_OPEN_FILES: Number of open files > _FS_SHARE\n");
            break;
        case FR_INVALID_PARAMETER:
            printf("FR_INVALID_PARAMETER: Given parameter is invalid\n");
            break;
        default:
            printf("Unknown error: %d\n", res);
            break;
    }
#endif
}


