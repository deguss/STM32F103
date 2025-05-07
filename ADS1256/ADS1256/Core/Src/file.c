#include "file.h"

FATFS FatFs;
FATFS* pfs;


FRESULT readWriteConfigFile(const char* fname){
	char value[64]; // Buffer to store the value
	char uid[25];
	char str[36];
	FRESULT fres;
	getSTM_UID(uid);
	snprintf(str, sizeof(str),"MCU UID = %s\n", uid);
	ITM_SendString(str);

	//check if file present and UID matches
	fres = read_config(fname, "UID", value, sizeof(value));
	if (fres != FR_OK  || memcmp(uid, value, strlen(uid)) != 0) {
		ITM_SendString("config file not present or wrong UID!\n");
		ITM_SendString("creating new config file\n");
		fres = write_config(fname, "UID", uid);
		if (fres != FR_OK)
			return fres;
		fres = write_config(fname, "CH", "1");
		if (fres != FR_OK)
			return fres;
		fres = write_config(fname, "SPS", "10");
		if (fres != FR_OK){
			ITM_SendString("failed to create config file\n");
			return fres;
		}


	}

	fres = read_config(fname, "CH", value, sizeof(value));
	if (fres != FR_OK){
		ITM_SendString("failed to read config file\n");
		return fres;
	}
	adcDataArray.channels = check_range(atoi(value), 1, 2);
	ITM_SendString("CH=");
	snprintf(str, sizeof(str), "%u\n", adcDataArray.channels);
	ITM_SendString(str);

	fres = read_config(fname, "SPS", value, sizeof(value));
	if (fres != FR_OK){
		return fres;
	}
	uint16_t index = getSPSindex(validateSPS(atoi(value)));
	adcDataArray.sps = sps[index];
	adcDataArray.length = bufferSizes[index];
	ITM_SendString("SPS=");
	snprintf(str, sizeof(str), "%u\n", adcDataArray.sps);
	ITM_SendString(str);
	SD_state = SD_OK;

	return FR_OK;
}

FRESULT stat_with_lfn(const char *path, FILINFO *fno) {
#if _USE_LFN
    static char lfn[_MAX_LFN + 1];  // Static = safe if no threading
    fno->lfname = lfn;
    fno->lfsize = sizeof(lfn);
#endif
    return f_stat(path, fno);
}

FRESULT arraytoFile(AdcDataArrayStruct *arr){

	FRESULT fres;
	FIL fil;
	char str[64];
	char base[32];
	char fn[32];
	UINT bytesWritten;
	dateTimeStruct dT;
	static dateTimeStruct startdT, lastdT;
	//TODO timing

	__disable_irq();
	dT = dateTimeNow;
	__enable_irq();

	//determine if last call of this function was more than 3 seconds ago
	if (datetime_diff(lastdT, dT) > 3){
		//if recording not yet ongoing (contdT will be different from dT)
		//if (memcmp(&startdT, &dT, sizeof(dateTimeStruct)) != 0) {

		startdT = dT;

		snprintf(base,sizeof(base),"%4d_%02d_%02d", dT.year, dT.month, dT.day);	//create folder name
		FILINFO fno;
		fres = stat_with_lfn(base, &fno); // Check if directory exists
		if (fres != FR_OK) {
		    // Directory does not exist; create it
			fres = f_mkdir(base);
			if (fres != FR_OK) {
				ITM_SendString("failed to create directory\n");
				return fres;
			}
		}

		// ----------- log file ------------------------
		snprintf(fn,sizeof(fn),"%s/%02u%02u.txt",base, dT.hours, dT.minutes);	//create log file name 1204.txt
		fres = stat_with_lfn(fn, &fno); // Check if file exists
		if (fres != FR_OK) {	//file does not exists,
			fres = f_open(&fil, fn, FA_CREATE_ALWAYS | FA_WRITE); //Create a new file or overwrite if it exists.
			if (fres != FR_OK) {
				ITM_SendString("failed to create log file\n");
				f_close(&fil);
				return fres;
			}
			f_close(&fil); //I have to close it in order for further writes to open it again.


			snprintf(str, sizeof(str), "%4u.%02u.%02u", dT.year, dT.month, dT.day);
			fres = write_config(fn, "start_day", str);
			if (fres != FR_OK) {
				return fres;
			}

			snprintf(str, sizeof(str), "%2u%02u", dT.hours, dT.minutes);
			fres = write_config(fn, "start_time", str);
			if (fres != FR_OK) {
				return fres;
			}

			snprintf(str, sizeof(str), "%lu", datetime_to_epoch(dT));
			fres = write_config(fn, "epoch", str);
			if (fres != FR_OK) {
				return fres;
			}


			snprintf(str, sizeof(str), "%d", arr->sps);
			fres = write_config(fn, "SPS", str);
			if (fres != FR_OK) {
				return fres;
			}

			snprintf(str, sizeof(str), "%d", arr->channels);
			fres = write_config(fn, "CH", str);
			if (fres != FR_OK) {
				return fres;
			}
		}

	}



	// ------------- binary data file --------------------------
	snprintf(fn,sizeof(fn),"%s/%02d%02d.bin",base, dT.hours, dT.minutes);	//create data file name
	fres = f_open(&fil, fn, FA_WRITE | FA_OPEN_ALWAYS ); // If the file does not exist → it is created.
	if (fres != FR_OK) {
		ITM_SendString("failed to create data file\n");
		return fres;
	}
	fres = f_lseek(&fil, f_size(&fil)); // Seek to the end
	if (fres != FR_OK) {
		ITM_SendString("failed to seek to the end of the file\n");
		f_close(&fil);
		return fres;
	}
	// Write the data to the file
	fres = f_write(&fil, arr->data, arr->length * sizeof(int32_t), &bytesWritten);
	if (fres != FR_OK || bytesWritten != arr->length * sizeof(int32_t)) {
		f_close(&fil);
		ITM_SendString("could not write all data to file\n");
		return fres;
	}
	f_close(&fil);

	lastdT = dT;

	uint32_t duration = datetime_diff(startdT, dT);
	snprintf(str,20,"%lus\n", duration);
	ITM_SendString(str);

	return FR_OK;
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


FRESULT write_config(const char* fname, const char* key, const char* value) {
    FIL fil;
    FRESULT fr;

    // Open the file if it exists, or create a new one if it doesn't.
    fr = f_open(&fil, fname, FA_OPEN_ALWAYS | FA_WRITE);
    if (fr == FR_OK) {
        // Move the file pointer to the end
        f_lseek(&fil, f_size(&fil));

        // Write the key-value pair to the file
        f_printf(&fil, "%s=%s\n", key, value); // Use f_printf for formatted output

        // flush
        f_sync(&fil);
    }
    // Close the file
    f_close(&fil);
    return fr; // Return the result
}



FRESULT checkSDusage(uint32_t* percent, uint32_t* total){

	FRESULT fres;
	DWORD fre_clust;
	float totalSpace, freeSpace, usedPercent;
	// Check free space
	fres = f_getfree("", &fre_clust, &pfs);
	if (fres != FR_OK){
		return fres;
	}

	totalSpace = (float)((pfs->n_fatent - 2) * pfs->csize * 0.5);
	freeSpace = (float)(fre_clust * pfs->csize * 0.5);
	usedPercent = 100.0 - (freeSpace*100 / totalSpace);
#ifdef DEBUG
	char str[64];
	snprintf(str, sizeof(str), "total: %9.0fkB\n", totalSpace);
	ITM_SendString(str);
	snprintf(str, sizeof(str),"free: %10.0fkB\n", freeSpace);
	ITM_SendString(str);
	snprintf(str, sizeof(str),"%2.2f%% free\n", freeSpace*100 / totalSpace);
	ITM_SendString(str);
	snprintf(str, sizeof(str),"%2.2f%% used\n", usedPercent);
	ITM_SendString(str);
#endif
	*total = MINIM(((uint32_t)(totalSpace / 1000 / 1000)), 100);
	*percent = MINIM(((uint32_t)(usedPercent)), 99);
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

