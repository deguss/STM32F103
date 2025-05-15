#ifndef __FATFS_SD_H
#define __FATFS_SD_H

/* Definitions for MMC/SDC command */
#define CMD0     (0x40 + 0)    /* GO_IDLE_STATE      0b0100 0000   0x40 */
#define CMD1     (0x40 + 1)    /* SEND_OP_COND       0b0100 0001   0x41 */
#define CMD8     (0x40 + 8)    /* SEND_IF_COND       0b0100 1000   0x48 */
#define CMD9     (0x40 + 9)    /* SEND_CSD           0b0100 1001   0x49 */
#define CMD10    (0x40 + 10)   /* SEND_CID           0b0100 1010   0x4A */
#define CMD12    (0x40 + 12)   /* STOP_TRANSMISSION  0b0100 1100   0x4C */
#define CMD16    (0x40 + 16)   /* SET_BLOCKLEN       0b0101 0000   0x50 */
#define CMD17    (0x40 + 17)   /* READ_SINGLE_BLOCK  0b0101 0001   0x51 */
#define CMD18    (0x40 + 18)   /* READ_MULTIPLE_BLOC 0b0101 0010   0x52 */
#define CMD23    (0x40 + 23)   /* SET_BLOCK_COUNT    0b0101 0111   0x57 */
#define CMD24    (0x40 + 24)   /* WRITE_BLOCK        0b0101 1000   0x58 */
#define CMD25    (0x40 + 25)   /* WRITE_MULTIPLE_BLO 0b0101 1001   0x59 */
#define CMD41    (0x40 + 41)   /* SEND_OP_COND (ACMD)0b0101 1101   0x5D */
#define CMD55    (0x40 + 55)   /* APP_CMD            0b0101 0111   0x37 */
#define CMD58    (0x40 + 58)   /* READ_OCR           0b0101 1010   0x5A */

/* MMC card type flags (MMC_GET_TYPE) */
#define CT_MMC		0x01		/* MMC ver 3 */
#define CT_SD1		0x02		/* SD ver 1 */
#define CT_SD2		0x04		/* SD ver 2 */
#define CT_SDC		0x06		/* SD */
#define CT_BLOCK	0x08		/* Block addressing */

/* Functions */
DSTATUS SD_disk_initialize (BYTE pdrv);
DSTATUS SD_disk_status (BYTE pdrv);
DRESULT SD_disk_read (BYTE pdrv, BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_write (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_ioctl (BYTE pdrv, BYTE cmd, void* buff);

#define SPI_TIMEOUT 100

extern SPI_HandleTypeDef 	hspi2;
#define HSPI_SDCARD		 	&hspi2
#define	SD_CS_PORT			GPIOB
#define SD_CS_PIN			GPIO_PIN_12



bool is_sd_inserted(void);



#endif
