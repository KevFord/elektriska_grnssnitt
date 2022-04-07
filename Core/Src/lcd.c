/*
 * lcd.c
 *
 *  Created on: Apr 27, 2021
 *      Author: axeand
 */
#include "lcd.h"
// D7 D6 D5 D4 BT E  RW RS

TIM_HandleTypeDef htim2;

void min_Delay(uint32_t Delay){
	uint16_t tickStart = __HAL_TIM_GET_COUNTER(&htim2);
	uint32_t wait = Delay;
  while ((__HAL_TIM_GET_COUNTER(&htim2) - tickStart) < wait)
  {
  }
}

void TextLCD_Strobe(TextLCDType *lcd) // anropas för att kunnna sktriva till LCD
{
	// Set bit 2 which corresponds to E (strobe) and send data
	lcd->data |= 0x04;
	HAL_I2C_Master_Transmit(lcd->hi2c, lcd->DevAddress, &lcd->data, 1, 1000);
	HAL_Delay(1);
	// Clear bit 2 which corresponds to E (strobe) and send data
	lcd->data &= 0xFB;
	HAL_I2C_Master_Transmit(lcd->hi2c, lcd->DevAddress, &lcd->data, 1, 1000);
//	min_Delay(40);
	HAL_Delay(1);
}

void TextLCD_Cmd(TextLCDType *lcd, uint8_t cmd) // skickar cmd till LCD, anrop: TextLCD_Cmd(&lcd, 0x02); där 02 är kommandot
{
	// for command RS is set to 0
	lcd->data = (lcd->data & 0x0C) | (cmd & 0xF0);
	HAL_I2C_Master_Transmit(lcd->hi2c, lcd->DevAddress, &lcd->data, 1, 1000);
	TextLCD_Strobe(lcd);
	lcd->data = (lcd->data & 0x0C) | (cmd << 4);
	HAL_I2C_Master_Transmit(lcd->hi2c, lcd->DevAddress, &lcd->data, 1, 1000);
	TextLCD_Strobe(lcd);
}

void TextLCD_Data(TextLCDType *lcd, uint8_t data) // anrop: TextLCD_Data(&lcd, 0x55); skickar komando 55 till LCD
{
	// for data RS is set to 1
	lcd->data = (lcd->data & 0x0D) | (data & 0xF0) | 0x01;
	HAL_I2C_Master_Transmit(lcd->hi2c, lcd->DevAddress, &lcd->data, 1, 1000);
	TextLCD_Strobe(lcd);
	lcd->data = (lcd->data & 0x0D) | (data << 4) | 0x01;
	HAL_I2C_Master_Transmit(lcd->hi2c, lcd->DevAddress, &lcd->data, 1, 1000);
	TextLCD_Strobe(lcd);
}

void TextLCD_Init(TextLCDType *lcd, I2C_HandleTypeDef *hi2c, uint8_t DevAddress)
{
	lcd->hi2c = hi2c;
	lcd->DevAddress = DevAddress;
	lcd->data = 0x38;

	// Do init setting LCD controller into 4-bit mode

	HAL_I2C_Master_Transmit(lcd->hi2c, lcd->DevAddress, &lcd->data, 1, 1000);
	TextLCD_Strobe(lcd);
	HAL_I2C_Master_Transmit(lcd->hi2c, lcd->DevAddress, &lcd->data, 1, 1000);
	TextLCD_Strobe(lcd);
	HAL_I2C_Master_Transmit(lcd->hi2c, lcd->DevAddress, &lcd->data, 1, 1000);
	TextLCD_Strobe(lcd);
	lcd->data = 0x28;
	HAL_I2C_Master_Transmit(lcd->hi2c, lcd->DevAddress, &lcd->data, 1, 1000);
	TextLCD_Strobe(lcd);

	// Finished setting up 4-bit mode. Let's configure display

	TextLCD_Cmd(lcd, 0x28); //N=1 (2 line), F=0 (5x8)
	TextLCD_Cmd(lcd, 0x08); //Display off, Cursor Off, Blink off
	TextLCD_Cmd(lcd, 0x01); //Clear
	HAL_Delay(5);
	TextLCD_Cmd(lcd, 0x06); //ID=1(increment), S=0 (no shift)
	TextLCD_Cmd(lcd, 0x0C); //Display on, Cursor Off, Blink off
}

void TextLCD_Home(TextLCDType *lcd){ // skall flytta cursor till första positionen

	TextLCD_Cmd(lcd, 0x02); // 02 = home
//	TextLCD_Strobe(lcd); // läs in-pin
}

void TextLCD_Clear(TextLCDType *lcd){

	TextLCD_Cmd(lcd, 0x01); // skickar 01 till LCD, 01 = clear. Rensar en pixel?
//	TextLCD_Strobe(lcd); // anropar för att LCD skall uppdateras och läsa in-pinnen.
}

void TextLCD_Position(TextLCDType *lcd, int x, int y) // Stulen.
{
	uint8_t cmd_bit = 0x80;
	uint8_t adr     = x + (0x40 * y);
	uint8_t cmd     = cmd_bit | adr;
	TextLCD_Cmd(lcd, cmd);
}

void TextLCD_Putchar(TextLCDType *lcd, uint8_t data){ // anrop: TextLCD_Putchar(&lcd, 'K'); ref till lcd och vilken char.

	TextLCD_Data(lcd, data);// + data);
//	TextLCD_Strobe(lcd);
}

void TextLCD_Putnr(TextLCDType *lcd, uint8_t data){ // anrop: TextLCD_Putchar(&lcd, 3); Skriver ut ett heltal mellan 0 - 9.

	TextLCD_Data(lcd, 0x30 + data);// + data);
//	TextLCD_Strobe(lcd);
}

void TextLCD_Puts(TextLCDType *lcd, char *string){ // anrop: liknande putchar, men en string "texttext" som andra argument.

	for(int i = 0; string[i] != 0; i++)
		TextLCD_Putchar(lcd, string[i]);
//	TextLCD_Strobe(lcd);
}
#if 0
void TextLCD_Printf(TextLCDType *lcd, char *message, …){

}
#endif
