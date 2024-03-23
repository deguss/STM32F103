#include <ads1256.h>

uint8_t readRegister(uint8_t reg){
    uint8_t dat;

    uint8_t command[2];
    command[0] = CMD_RREG_BASE | reg;
    command[1] = 0; // read (this + 1) registers

    HAL_SPI_Transmit(&hspi1, command, 2, HAL_MAX_DELAY); // Send register address and data
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    HAL_SPI_Receive(&hspi1, &dat, 1, HAL_MAX_DELAY); // read register
    return dat;
}


void sendCommand(uint8_t cmd){
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    ;
}


void writeRegister(uint8_t reg, uint8_t data){
    uint8_t txData[3];

    txData[0] = CMD_WREG_BASE | reg;
    txData[1] = 0;          // (this + 1) registers will be written (0+1=1)
    txData[2] = data;

    HAL_SPI_Transmit(&hspi1, txData, 3, HAL_MAX_DELAY); // Send register address and data
    ;
}


// Set up ADS1256 configuration registers
// Initialize SPI communication with ADS1256
// Configure the ADC settings
uint8_t setupADS1256() {

	HAL_Delay(100);
	sendCommand(CMD_RESET);
	HAL_Delay(100);

    // read the STATUS register
	uint8_t status = readRegister(REG_STATUS);
    writeRegister(REG_STATUS, (status & 0xF0) | 0x02); //enable buffer amplifier
    while(NOT_DRDY) ;	//wait for calibration to complete

    // Configure the DRATE register for desired data rate
    writeRegister(REG_DRATE, SPS_50); // Set data rate to 50Hz
    while(NOT_DRDY) ;	//wait to complete

    // Configure the ADCON register
    writeRegister(REG_ADCON, PGA1 ); // Set PGA gain to 1
    while(NOT_DRDY) ;	//wait to complete
    
    sendCommand(CMD_SELFCAL); //start Offset and Gain Self-Calibration
    //can last up to 2 seconds for low data rates
    while(NOT_DRDY) ;	//wait for calibration to complete


    return status;
}

void stopSampling(void){
    while(NOT_DRDY) ;	//wait to complete
    sendCommand(CMD_SDATAC);
    while(NOT_DRDY) ;	//wait to complete
    //sendCommand(CMD_RESET);
}

// Channel Switching for differential mode. Use -1 to set input channel to
// AINCOM
uint8_t setChannel(int8_t AIN_P, int8_t AIN_N) {
	uint8_t MUX_CHANNEL;
	uint8_t MUXP;
	uint8_t MUXN;

	switch (AIN_P) {
		case 0:
		  MUXP = MUXP_AIN0;
		  break;
		case 1:
		  MUXP = MUXP_AIN1;
		  break;
		case 2:
		  MUXP = MUXP_AIN2;
		  break;
		case 3:
		  MUXP = MUXP_AIN3;
		  break;
		case 4:
		  MUXP = MUXP_AIN4;
		  break;
		case 5:
		  MUXP = MUXP_AIN5;
		  break;
		case 6:
		  MUXP = MUXP_AIN6;
		  break;
		case 7:
		  MUXP = MUXP_AIN7;
		  break;
		default:
		  MUXP = MUXP_AINCOM;
	}

	switch (AIN_N) {
		case 0:
		  MUXN = MUXN_AIN0;
		  break;
		case 1:
		  MUXN = MUXN_AIN1;
		  break;
		case 2:
		  MUXN = MUXN_AIN2;
		  break;
		case 3:
		  MUXN = MUXN_AIN3;
		  break;
		case 4:
		  MUXN = MUXN_AIN4;
		  break;
		case 5:
		  MUXN = MUXN_AIN5;
		  break;
		case 6:
		  MUXN = MUXN_AIN6;
		  break;
		case 7:
		  MUXN = MUXN_AIN7;
		  break;
		default:
		  MUXN = MUXN_AINCOM;
	}

	MUX_CHANNEL = MUXP | MUXN;

    while(NOT_DRDY) ; 	//wait to be ready
    writeRegister(REG_MUX, MUX_CHANNEL);
    if (readRegister(REG_MUX) == MUX_CHANNEL){
    	return 0;
    }
    return 1;

}

void setGain(uint8_t drate, uint8_t gain) {
	sendCommand(CMD_SDATAC);  // send out SDATAC command to stop continuous reading mode.

	writeRegister(REG_DRATE, drate);  // write data rate register
	while(NOT_DRDY) ;	//wait to complete


	writeRegister(REG_ADCON, 0x07 & gain);
	while(NOT_DRDY) ;	//wait to complete

    sendCommand(CMD_SELFCAL); //start Offset and Gain Self-Calibration
    //can last up to 2 seconds for low data rates
    while(NOT_DRDY) ;	//wait to complete

    sendCommand(CMD_RDATAC);
}

int32_t concatenateToInt32(uint8_t *adcData){
	uint8_t highbyte = *adcData;
	uint8_t midbyte = *(adcData + 1);
	uint8_t lowbyte = *(adcData + 2);

    // Concatenate the bytes in the correct order to form a single integer value
    int adcValue = ((int32_t)highbyte << 16) + ((int32_t)midbyte << 8) + (int32_t)lowbyte;
    if (adcValue & 0x00800000) {	//transfer sign bit from pos. 23 to pos. 31
    	adcValue |= 0xff000000;
    }
    return adcValue;
}
