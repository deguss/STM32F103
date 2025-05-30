/*
 * lcd.c
 *
 *  Created on: 10/06/2018
 *      Author: Olivier Van den Eede
 */

#include "lcd.h"
#include "main.h"

Lcd_HandleTypeDef lcd;


const uint8_t ROW_16[] = {0x00, 0x40, 0x10, 0x50};
const uint8_t ROW_20[] = {0x00, 0x40, 0x14, 0x54};
/************************************** Static declarations **************************************/

static void lcd_write_command(Lcd_HandleTypeDef * lcd, uint8_t command);
static void lcd_write(Lcd_HandleTypeDef * lcd, uint8_t data, uint8_t len);
void lcd_write_fast(Lcd_HandleTypeDef * lcd, uint8_t data, uint8_t len);

/************************************** Function definitions **************************************/

/**
 * Create new Lcd_HandleTypeDef and initialize the Lcd
 */
Lcd_HandleTypeDef lcd_create(
		Lcd_PortType port[], Lcd_PinType pin[],
		Lcd_PortType rs_port, Lcd_PinType rs_pin,
		Lcd_PortType en_port, Lcd_PinType en_pin, Lcd_ModeTypeDef mode)
{
	Lcd_HandleTypeDef lcd;

	lcd.mode = mode;

	lcd.en_pin = en_pin;
	lcd.en_port = en_port;

	lcd.rs_pin = rs_pin;
	lcd.rs_port = rs_port;

	lcd.data_pin = pin;
	lcd.data_port = port;

	lcd_init(&lcd);

	return lcd;
}

/**
 * Initialize 16x2-lcd without cursor
 */
void lcd_init(Lcd_HandleTypeDef * lcd)
{
	if(lcd->mode == LCD_4_BIT_MODE)
	{
			lcd_write_command(lcd, 0x33);
			lcd_write_command(lcd, 0x32);
			lcd_write_command(lcd, FUNCTION_SET | OPT_N);				// 4-bit mode
	}
	else
		lcd_write_command(lcd, FUNCTION_SET | OPT_DL | OPT_N);


	lcd_write_command(lcd, CLEAR_DISPLAY);						// Clear screen
	lcd_write_command(lcd, DISPLAY_ON_OFF_CONTROL | OPT_D);		// Lcd-on, cursor-off, no-blink
	lcd_write_command(lcd, ENTRY_MODE_SET | OPT_INC);			// Increment cursor
}

/**
 * Write a number on the current position
 */
void lcd_int(Lcd_HandleTypeDef * lcd, int number)
{
	char buffer[11];
	sprintf(buffer, "%d", number);

	lcd_string(lcd, buffer);
}


/**
 * Write a string on the current position
 */
void lcd_string(Lcd_HandleTypeDef * lcd, char * string)
{
	uint8_t i;
	for(i=0; i<strlen(string); i++){
		lcd_write_data(lcd, string[i]);
	}
}

/**
 * Write a string on the current position and clear the rest of the line
 */
void lcd_line(Lcd_HandleTypeDef * lcd, char * string)
{
	uint8_t i;
	for(i=0; i<strlen(string); i++){
		lcd_write_data(lcd, string[i]);
	}

	#ifdef LCD20xN
	while (i<20){
		lcd_write_data(lcd, ' ');
		i++;
	}
	#endif

	#ifdef LCD16xN
	while (i<16){
		lcd_write_data(lcd, ' ');
		i++;
	}
	#endif

}

/**
 * Set the cursor position
 */
void lcd_cursor(Lcd_HandleTypeDef * lcd, uint8_t row, uint8_t col)
{
	#ifdef LCD20xN
	lcd_write_command(lcd, SET_DDRAM_ADDR + ROW_20[row] + col);
	#endif

	#ifdef LCD16xN
	lcd_write_command(lcd, SET_DDRAM_ADDR + ROW_16[row] + col);
	#endif
}

/**
 * Clear the screen
 */
void lcd_clear(Lcd_HandleTypeDef * lcd) {
	lcd_write_command(lcd, CLEAR_DISPLAY);
	DELAY(1);
}

void lcd_define_char(Lcd_HandleTypeDef * lcd, uint8_t code, uint8_t bitmap[]){
	lcd_write_command(lcd, SETCGRAM_ADDR + (code << 3));
	for(uint8_t i=0;i<8;++i){
		lcd_write_data(lcd, bitmap[i]);
	}

}


/************************************** Static function definition **************************************/

/**
 * Write a byte to the command register
 */
void lcd_write_command(Lcd_HandleTypeDef * lcd, uint8_t command)
{
	HAL_GPIO_WritePin(lcd->rs_port, lcd->rs_pin, LCD_COMMAND_REG);		// Write to command register

	if(lcd->mode == LCD_4_BIT_MODE)
	{
		lcd_write(lcd, (command >> 4), LCD_NIB);
		lcd_write(lcd, command & 0x0F, LCD_NIB);
	}
	else
	{
		lcd_write(lcd, command, LCD_BYTE);
	}

}

/**
 * Write a byte to the data register
 */
void lcd_write_data(Lcd_HandleTypeDef * lcd, uint8_t data)
{
	HAL_GPIO_WritePin(lcd->rs_port, lcd->rs_pin, LCD_DATA_REG);			// Write to data register

	if(lcd->mode == LCD_4_BIT_MODE)
	{
		lcd_write_fast(lcd, data >> 4, LCD_NIB);
		lcd_write_fast(lcd, data & 0x0F, LCD_NIB);
	}
	else
	{
		lcd_write_fast(lcd, data, LCD_BYTE);
	}

}

// used only for data writing (short strobe delay)
void lcd_write_fast(Lcd_HandleTypeDef * lcd, uint8_t data, uint8_t len)
{
	HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin, 1);
	for(uint8_t i = 0; i < len; i++)
	{
		HAL_GPIO_WritePin(lcd->data_port[i], lcd->data_pin[i], (data >> i) & 0x01);
	}
	for(volatile int i=0; i<72*5; i++)
		__NOP();
	HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin, 0); 		// Data receive on falling edge
}


/**
 * Set len bits on the bus and toggle the enable line
 */
void lcd_write(Lcd_HandleTypeDef * lcd, uint8_t data, uint8_t len)
{
	HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin, 1);
	for(uint8_t i = 0; i < len; i++)
	{
		HAL_GPIO_WritePin(lcd->data_port[i], lcd->data_pin[i], (data >> i) & 0x01);
	}
	/* for(int i=0; i<72*200; i++) //200us delay minimum
		__NOP();
	*/
	HAL_Delay(1);
	HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin, 0); 		// Data receive on falling edge
}
