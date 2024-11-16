#include "file.h"

FATFS FatFs;
FATFS* pfs;


void getSTM_UID(char *uid) {
	uint32_t id[3];
    // Ensure that the address is aligned to a uint32_t pointer
    id[0] = *((__IO uint32_t*)0x1FFFF7E8); // Read the first word
    id[1] = *((__IO uint32_t*)0x1FFFF7EC); // Read the second word
    id[2] = *((__IO uint32_t*)0x1FFFF7F0); // Read the third word
    snprintf(uid, 25, "%08X%08X%08X", (unsigned int)id[0], (unsigned int)id[1], (unsigned int)id[2]);
}

//f_sync(&fil);

FRESULT read_config(const char* key, char* value, size_t value_size) {
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
    fr = f_open(&fil, CONFIG_FILE_NAME, FA_READ);
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


FRESULT write_config(const char* key, const char* value) {
    FIL fil;
    FRESULT fr;

    // Open the file for writing (create if it doesn't exist)
    fr = f_open(&fil, CONFIG_FILE_NAME, FA_OPEN_ALWAYS | FA_WRITE);
    if (fr == FR_OK) {
        // Move the file pointer to the end
        f_lseek(&fil, f_size(&fil));

        // Write the key-value pair to the file
        f_printf(&fil, "%s=%s\n", key, value); // Use f_printf for formatted output

        // Close the file
        f_close(&fil);
    }
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
	*total = MIN(((uint32_t)(totalSpace / 1000 / 1000)), 100);
	*percent = MIN(((uint32_t)(usedPercent)), 99);
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

