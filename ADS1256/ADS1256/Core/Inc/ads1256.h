/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADS1256_H
#define __ADS1256_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"


#define NOT_DRDY   HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)

// Define the ADS1256 register addresses
#define REG_STATUS 0x00
#define REG_MUX    0x01
#define REG_ADCON  0x02
#define REG_DRATE  0x03
#define REG_IO     0x04

// define some constants for the ADCON register
#define PGA1 0x00
#define PGA2 0x01
#define PGA4 0x02
#define PGA8 0x03
#define PGA16 0x04
#define PGA32 0x05
#define PGA64 0x06



// Define the SPS options using enum for better readability
typedef enum {
    SPS_2_5 = 0x03,
    SPS_5   = 0x13,
    SPS_10  = 0x23,
    SPS_15  = 0x33,
    SPS_25  = 0x43,
    SPS_30  = 0x53,
    SPS_50  = 0x63,
    SPS_60  = 0x72,
    SPS_100 = 0x82,
    SPS_500 = 0x92,
    SPS_1000 = 0xA1,
    SPS_2000 = 0xB0,
    SPS_3750 = 0xC0,
    SPS_7500 = 0xD0,
    SPS_15000 = 0xE0,
    SPS_30000 = 0xF0
} SPS_OPTIONS;

#define NUM_SPS_OPTIONS 16
extern const uint16_t sps[NUM_SPS_OPTIONS];

// define some commands
#define CMD_WAKEUP               0x00
#define CMD_RDATA                0x01
#define CMD_RDATAC               0x03
#define CMD_SDATAC               0x0F
#define CMD_RREG_BASE            0x10
#define CMD_WREG_BASE            0x50
#define CMD_SELFCAL              0xF0
#define CMD_SELFOCAL             0xF1
#define CMD_SELFGCAL             0xF2
#define CMD_SYSOCAL              0xF3
#define CMD_SYSGCAL              0xF4
#define CMD_SYNC                 0xFC
#define CMD_STANDBY              0xFD
#define CMD_RESET                0xFE

// define multiplexer codes
#define MUXP_AIN0 0x00
#define MUXP_AIN1 0x10
#define MUXP_AIN2 0x20
#define MUXP_AIN3 0x30
#define MUXP_AIN4 0x40
#define MUXP_AIN5 0x50
#define MUXP_AIN6 0x60
#define MUXP_AIN7 0x70
#define MUXP_AINCOM 0x80

#define MUXN_AIN0 0x00
#define MUXN_AIN1 0x01
#define MUXN_AIN2 0x02
#define MUXN_AIN3 0x03
#define MUXN_AIN4 0x04
#define MUXN_AIN5 0x05
#define MUXN_AIN6 0x06
#define MUXN_AIN7 0x07
#define MUXN_AINCOM 0x08

#define NUM_PGA_OPTIONS 7
extern const uint8_t pga[NUM_PGA_OPTIONS];
extern const uint16_t range[NUM_PGA_OPTIONS];
extern const uint8_t pga_const[NUM_PGA_OPTIONS];


extern SPI_HandleTypeDef hspi1;

int32_t calculate_average(const int32_t *Array, size_t length);
uint8_t check_range(int value, uint8_t LLIM, uint8_t ULIM);
uint16_t getSPSindex(int input);
uint16_t validateSPS(int input);
uint8_t validatePGA(int input);
uint8_t getPGAindex(int input);

uint8_t getSPSRegValue(int input);
void sendCommand(uint8_t cmd);
void writeRegister(uint8_t reg, uint8_t value);
uint8_t readRegister(uint8_t reg);
//call this to initialize ADC. It will not start sampling. Use setGain afterwards
uint8_t setupADS1256(void);
void stopSampling(void);

//sets channel on the fly without interrupting continuous AD conversion
// for differential mode use any combination [0,7]. Use value of -1 to set input channel to AINCOM (single ended)
uint8_t setChannel(int8_t AIN_P, int8_t AIN_N);

//stops conversion, sets parameters and restart AD conversion with new parameters in continuous mode
//valid datarate values are listed in definition above: SPS_30000 SPS15000 .. SPS_2_5
//valid gain values are listed in definition above: PGA1 PGA2 .. PGA64
void setGain(uint8_t drate, uint8_t gain);

int32_t concatenateToInt32(uint8_t *adcData);

#endif
