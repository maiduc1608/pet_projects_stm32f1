#ifndef MYLCD_H
#define MYLCD_H

#include <stm32f10x.h>
#include <stdio.h>
#include <string.h>
#include "myDelay.h"
void I2C1_Init(void);
void I2C1_Start(void);
void I2C1_Stop(void);
void I2C1_Write(uint8_t data);
void I2C1_Send_Address(uint8_t address);
void I2C1_Write_Buffer(uint8_t address, uint8_t *buffer, uint8_t length);

void LCD_I2C_Write_CMD(uint8_t data);
void LCD_I2C_Write_DATA(uint8_t data);
void LCD_I2C_Init(void);
void LCD_I2C_Clear(void);
void LCD_I2C_Location(uint8_t x, uint8_t y);
void LCD_I2C_Write_String(char* string);
void LCD_I2C_Write_Number(int number);

#endif