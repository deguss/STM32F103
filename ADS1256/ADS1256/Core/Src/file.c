#include "file.h"
#include "main.h"
#include "displays.h"

FATFS FatFs;
FATFS* pfs;


FRESULT readWriteConfigFile(const char* fname, bool overwrite){
	char value[64]; // Buffer to store the value
	char uid[25];
	char str[36];
	FRESULT fres;
	getSTM_UID(uid);
	snprintf(str, sizeof(str),"MCU UID = %s\n", uid);
	ITM_SendString(str);


	fres = read_config(fname, "UID", value, sizeof(value)); //check if file present and UID matches
	if (fres != FR_OK  || memcmp(uid, value, strlen(uid)) != 0 || overwrite) {
		ITM_SendString("config file not present or wrong UID or requested to overwrite!\n");

		fres = write_config(fname, overwrite, "UID", uid);	//write new file
		if (fres != FR_OK)	 	goto err_write;
		snprintf(str, sizeof(str), "%u", config_channels);
		fres = write_config(fname, false, "channels", str);	//append to newly created file
		if (fres != FR_OK) 		goto err_write;
		snprintf(str, sizeof(str), "%u", sps[sps_index]);
		fres = write_config(fname, false, "sps", str);
		if (fres != FR_OK) 		goto err_write;
		snprintf(str, sizeof(str), "%u", pga[pga_index]);
		fres = write_config(fname, false, "gain", str);
		if (fres != FR_OK) 		goto err_write;

	}

	//read back

	fres = read_config(fname, "channels", value, sizeof(value));
	if (fres != FR_OK)		goto err_close;
	config_channels = check_range(atoi(value), 1, 2);
	ITM_SendString("channels=");
	snprintf(str, sizeof(str), "%u\n", config_channels);
	ITM_SendString(str);

	fres = read_config(fname, "sps", value, sizeof(value));
	if (fres != FR_OK)		goto err_close;
	sps_index = getSPSindex(validateSPS(atoi(value)));
	adcDataArray[0].sps = sps[sps_index];
	adcDataArray[0].length = bufferSizes[sps_index];
	adcDataArray[1].sps = sps[sps_index];
	adcDataArray[1].length = bufferSizes[sps_index];
	ITM_SendString("sps=");
	snprintf(str, sizeof(str), "%u\n", adcDataArray[0].sps);
	ITM_SendString(str);

	fres = read_config(fname, "gain", value, sizeof(value));
	if (fres != FR_OK)		goto err_close;
	pga_index = getPGAindex(validatePGA(atoi(value)));
	adcDataArray[0].gain = pga[pga_index];
	adcDataArray[1].gain = pga[pga_index];
	ITM_SendString("gain=");
	snprintf(str, sizeof(str), "%u\n", adcDataArray[0].gain);
	ITM_SendString(str);


	SD_state = SD_OK;
	return FR_OK;

err_close:
	ITM_SendString("failed to read config\n");
	return fres;

err_write:
	ITM_SendString("failed to write config\n");
	return fres;
}

FRESULT stat_with_lfn(const char *path, FILINFO *fno) {
#if _USE_LFN
    static char lfn[_MAX_LFN + 1];  // Static = safe if no threading
    fno->lfname = lfn;
    fno->lfsize = sizeof(lfn);
#endif
    return f_stat(path, fno);
}

FRESULT writeArrayToFile(AdcDataArrayStruct *arr, dateTimeStruct* startdT) {
    FRESULT fres;
    static FIL fil;
    UINT bytesWritten;
    FILINFO fno;
    static bool fileOpen = false;
    static DWORD current_offset = 0;
    volatile uint32_t timer_perf, timer_max=0;
    char str[64];
    static char base[13+1]; 	// YYYY-MM-DD
    static char fn[6+1];		// HHMM
    static char path[25];	// YYYY-MM-DD/HHMM.bin
    static dateTimeStruct lastdT, dT;

    timer_perf = HAL_GetTick();

    __disable_irq();
    dT = dateTimeNow;
    __enable_irq();

    if (ADS_state == ADS_READY) {
        if (dT.seconds != 0 || dT.minutes%2!=0) {
            return FR_OK; // Wait for full minute and minute MOD 5 = 0
        }
        ITM_SendString("start recording\n");
        ADS_state = ADS_RECORDING;
    }

    arr->epochtime = datetime_to_epoch(dT);

    // Check for file rotation
    bool isNewSession = datetime_diff(lastdT, dT) > 5 ||
                        (dT.hours == 0 && dT.minutes == 0 && dT.seconds == 0);

    if (isNewSession) {
        *startdT = dT;
        snprintf(base, sizeof(base), "%4u-%02u-%02u", dT.year, dT.month, dT.day);
        snprintf(fn, sizeof(fn), "%02u%02u", startdT->hours, startdT->minutes);

        // Ensure directory exists
        fres = stat_with_lfn(base, &fno);
        if (fres != FR_OK && f_mkdir(base) != FR_OK) {
            ITM_SendString("failed to create dir\n");
            return fres;
        }

        // Create log file if needed
        snprintf(path, sizeof(path), "%s/%s.txt", base, fn);
        if (stat_with_lfn(path, &fno) != FR_OK) {
            fres = f_open(&fil, path, FA_CREATE_ALWAYS | FA_WRITE);
            if (fres != FR_OK) {
                ITM_SendString("failed to create log\n");
                return fres;
            }

            snprintf(str, sizeof(str), "%4u-%02u-%02u", dT.year, dT.month, dT.day);
            if (write_config_line(&fil, "start_day", str) != FR_OK) goto error_close;

            snprintf(str, sizeof(str), "%02u:%02u:%02u", dT.hours, dT.minutes, dT.seconds);
            if (write_config_line(&fil, "start_time", str) != FR_OK) goto error_close;

            snprintf(str, sizeof(str), "%lu", arr->epochtime);
            if (write_config_line(&fil, "epochtime", str) != FR_OK) goto error_close;

            snprintf(str, sizeof(str), "%u", arr->sps);
            if (write_config_line(&fil, "sps", str) != FR_OK) goto error_close;

            snprintf(str, sizeof(str), "%u", config_channels);
            if (write_config_line(&fil, "channels", str) != FR_OK) goto error_close;

            snprintf(str, sizeof(str), "%u", arr->gain);
            if (write_config_line(&fil, "ch1_gain", str) != FR_OK) goto error_close;

            f_sync(&fil);
            f_close(&fil);
        }
        snprintf(path, sizeof(path), "%s/%s.bin", base, fn);	// create binary file name

        if (fileOpen)
        	f_close(&fil);											// takes 21ms

        fres = f_open(&fil, path, FA_WRITE | FA_OPEN_ALWAYS);		// takes appr. 40ms
        if (fres != FR_OK) {
        	fileOpen=false;
            ITM_SendString("failed to create data\n");
            return fres;
        }
        fileOpen=true;

        fres = f_lseek(&fil, f_size(&fil));						// takes appr. 10-20ms depending on fsize
        if (fres != FR_OK) {
            ITM_SendString("failed to seek\n");
            goto error_close;
        }

    }


    // Write binary data
    size_t write_len = 12 + arr->length * sizeof(int32_t); // Header + data
    fres = f_write(&fil, arr, write_len, &bytesWritten); 	//7-11ms
    if (fres != FR_OK || bytesWritten != write_len) {
        ITM_SendString("could not write all data\n");
        goto error_close;
    }
    current_offset += write_len;

    lastdT = dT;

    if (dT.seconds == 0){	//write file to disk every full minute
		f_sync(&fil);
		//sprintf(str,"%02u%02u: %u\n", dT.hours, dT.minutes, timer_max);
		//ITM_SendString(str);
	}

    if ((HAL_GetTick() - timer_perf) > timer_max){
    	timer_max = (HAL_GetTick() - timer_perf);
    }

    return FR_OK;

error_close:
    f_close(&fil);
    return fres;
}



void getSTM_UID(char *uid) {
	uint32_t id[3];
    // Ensure that the address is aligned to a uint32_t pointer
    id[0] = *((__IO uint32_t*)0x1FFFF7E8); // Read the first word
    id[1] = *((__IO uint32_t*)0x1FFFF7EC); // Read the second word
    id[2] = *((__IO uint32_t*)0x1FFFF7F0); // Read the third word
    snprintf(uid, 25, "%08X%08X%08X", (unsigned int)id[0], (unsigned int)id[1], (unsigned int)id[2]);
}

//f_sync(&fil);

FRESULT read_config(const char* fname, const char* key, char* value, size_t value_size) {
    FIL fil;
    FRESULT fr;
    char line[128]; // Buffer for reading lines

    // Check if value buffer is valid
    if (value == NULL || value_size == 0) {
        // Handle error appropriately, return an error code if necessary
        return FR_INVALID_PARAMETER; // Custom error handling
    }

    //initialize value to prevent garbage data
    memset(value, 0, value_size);

    // Open the file for reading
    fr = f_open(&fil, fname, FA_READ);
    if (fr != FR_OK) { //file does not exists
    	//printf("Config file does not exist!\n");
    	ITM_SendString("Error opening file!\n");
    	return fr;
    }

	// Read lines
	while (f_gets(line, sizeof(line), &fil)) {
		// Check if the line starts with the key
		if (strncmp(line, key, strlen(key)) == 0 && line[strlen(key)] == '=') {
			// Ensure space is available to copy
			size_t copy_length = strlen(line + strlen(key) + 1);
			if (copy_length < value_size) {
				strncpy(value, line + strlen(key) + 1, value_size - 1); // Copy allowed size
				value[value_size - 1] = '\0'; // Ensure null-termination
			} else {
				// Handle the case where it exceeds value_size
				strncpy(value, line + strlen(key) + 1, value_size - 1);
				value[value_size - 1] = '\0'; // Null-terminate
			}
			break; // Exit the loop after copying the value
		}
	}
	f_close(&fil); // Close the file
	return FR_OK;  // Return success if everything goes fine
}

FRESULT write_config_line(FIL* fil, const char* key, const char* value) {
    FRESULT fr;
    UINT bytesWritten;
    char line[128]; // Buffer for writing

    if (f_size(fil)){ //file with some content
        // Move the file pointer to the end
        fr = f_lseek(fil, f_size(fil));
        if (fr != FR_OK){
        	f_close(fil);
        	return fr;
        }
    }

    // create the key-value pair into a buffer
    snprintf(line, sizeof(line), "%s=%s\n", key, value);

    fr = f_write(fil, line, strlen(line), &bytesWritten);
    if (fr != FR_OK || bytesWritten != strlen(line)) {
        f_close(fil);
        return fr;
    }

    return fr; // Return the result
}

FRESULT write_config(const char* fname, bool overwrite, const char* key, const char* value) {
    FIL fil;
    FRESULT fr;
    UINT bytesWritten;
    char line[128]; // Buffer for writing

    BYTE mode = overwrite ? FA_CREATE_ALWAYS : FA_OPEN_ALWAYS;
    // Open the file if it exists, or create a new one if it doesn't.
    fr = f_open(&fil, fname, mode | FA_WRITE);
    if (fr != FR_OK){
    	return fr;
    }
    if (f_size(&fil)){ //file with some content
        // Move the file pointer to the end
        fr = f_lseek(&fil, f_size(&fil));
        if (fr != FR_OK){
        	f_close(&fil);
        	return fr;
        }
    }
    // create the key-value pair into a buffer
    snprintf(line, sizeof(line), "%s=%s\n", key, value);

    fr = f_write(&fil, line, strlen(line), &bytesWritten);
    if (fr != FR_OK || bytesWritten != strlen(line)) {
        f_close(&fil);
        return fr;
    }

	f_sync(&fil); 	// flush
    f_close(&fil); // Close the file
    return fr; // Return the result
}



FRESULT checkSDusage(uint32_t* percent, uint32_t* total) {
    FRESULT fres;
    DWORD fre_clust;
    DWORD total_clust, free_kB, total_kB, used_percent;

    // Get free cluster count
    fres = f_getfree("", &fre_clust, &pfs);
    if (fres != FR_OK) {
        return fres;
    }

    total_clust = pfs->n_fatent - 2;

    // Each cluster = pfs->csize * 512 bytes
    // Convert to kilobytes: (clusters * csize * 512) / 1024 = clusters * csize / 2
    total_kB = total_clust * pfs->csize / 2;
    free_kB  = fre_clust   * pfs->csize / 2;

    if (total_kB == 0) {
        *percent = 0;
        *total = 0;
        return fres;
    }

    // Calculate approximate used percent: (used * 100) / total
    used_percent = ((total_kB - free_kB) * 100) / total_kB;

/*
    char str[64];
    snprintf(str, sizeof(str), "total: %lu kB\n", (unsigned long)total_kB);
    ITM_SendString(str);
    snprintf(str, sizeof(str), "free:  %lu kB\n", (unsigned long)free_kB);
    ITM_SendString(str);
    snprintf(str, sizeof(str), "%lu%% free\n", (unsigned long)(100 - used_percent));
    ITM_SendString(str);
    snprintf(str, sizeof(str), "%lu%% used\n", (unsigned long)used_percent);
    ITM_SendString(str);
*/

    // Return rough values (optional clamping)
    *total = MIN(total_kB / 1024 / 1024, 100);  // total in GB, clamped to 100
    *percent = MIN(used_percent, 99);          // max 99% to prevent overflow

    return fres;
}


#define BUFFER_SIZE 20 // Fixed buffer size

void print_fresult(FRESULT res, char* buffer) {
    // Check if the provided buffer is not NULL
    if (buffer == NULL) {
        return;  // Early return if buffer is invalid
    }

    // Format the output string while ensuring it fits within the buffer
    switch (res) {
        case FR_OK:
            snprintf(buffer, BUFFER_SIZE, "FR_OK:%02d", res);
            break;
        case FR_DISK_ERR:
            snprintf(buffer, BUFFER_SIZE, "DISK_ERR:%02d", res); // FR_DISK_ERR: A hard error occurred in the low level disk I/O layer
            break;
        case FR_INT_ERR:
            snprintf(buffer, BUFFER_SIZE, "INT_ERR:%02d", res); // FR_INT_ERR: Assertion failed
            break;
        case FR_NOT_READY:
            snprintf(buffer, BUFFER_SIZE, "NOT_READY:%02d", res); // FR_NOT_READY: The volume is not ready
            break;
        case FR_NO_FILE:
            snprintf(buffer, BUFFER_SIZE, "NO_FILE:%02d", res); // FR_NO_FILE: Could not find the file
            break;
        case FR_NO_PATH:
            snprintf(buffer, BUFFER_SIZE, "NO_PATH:%02d", res); // FR_NO_PATH: Could not find the path
            break;
        case FR_INVALID_NAME:
            snprintf(buffer, BUFFER_SIZE, "INV_NAME:%02d", res); // FR_INVALID_NAME: The name format is invalid
            break;
        case FR_DENIED:
            snprintf(buffer, BUFFER_SIZE, "ACCESS_DENIED:%02d", res); // FR_DENIED: Access denied due to prohibited access
            break;
        case FR_EXIST:
            snprintf(buffer, BUFFER_SIZE, "FILE_EXISTS:%02d", res); // FR_EXIST: The file/dir already exists
            break;
        case FR_INVALID_OBJECT:
            snprintf(buffer, BUFFER_SIZE, "INV_OBJECT:%02d", res); // FR_INVALID_OBJECT: The file object is invalid
            break;
        case FR_WRITE_PROTECTED:
            snprintf(buffer, BUFFER_SIZE, "WR_PROTECTED:%02d", res); // FR_WRITE_PROTECTED: The media is write protected
            break;
        case FR_INVALID_DRIVE:
            snprintf(buffer, BUFFER_SIZE, "INV_DRIVE:%02d", res); // FR_INVALID_DRIVE: The specified drive is invalid
            break;
        case FR_NOT_ENABLED:
            snprintf(buffer, BUFFER_SIZE, "NOT_ENABLED:%02d", res); // FR_NOT_ENABLED: The volume has not been enabled
            break;
        case FR_NO_FILESYSTEM:
            snprintf(buffer, BUFFER_SIZE, "NO_FILESYS:%02d", res); // FR_NO_FILESYSTEM: No valid file system
            break;
        case FR_MKFS_ABORTED:
            snprintf(buffer, BUFFER_SIZE, "MKFS_ABORTED:%02d", res); // FR_MKFS_ABORTED: The mkfs operation has been aborted
            break;
        case FR_TIMEOUT:
            snprintf(buffer, BUFFER_SIZE, "TIMEOUT:%02d", res); // FR_TIMEOUT: Timeout occurred
            break;
        case FR_LOCKED:
            snprintf(buffer, BUFFER_SIZE, "LOCKED:%02d", res); // FR_LOCKED: The file or directory is locked
            break;
        case FR_NOT_ENOUGH_CORE:
            snprintf(buffer, BUFFER_SIZE, "NO_ENOUGH_MEM:%02d", res); // FR_NOT_ENOUGH_CORE: Not enough core memory
            break;
        case FR_TOO_MANY_OPEN_FILES:
            snprintf(buffer, BUFFER_SIZE, "TOO_MANY_FILES:%02d", res); // FR_TOO_MANY_OPEN_FILES: Too many files are open
            break;
        case FR_INVALID_PARAMETER:
            snprintf(buffer, BUFFER_SIZE, "INV_PARAM:%02d", res); // FR_INVALID_PARAMETER: Invalid parameter passed
            break;
        default:
            snprintf(buffer, BUFFER_SIZE, "UNKNOWN_ERR:%02d", res); // Full: Unknown error
            break;
    }

#ifdef DEBUG
    ITM_SendString(buffer);
#endif
}

