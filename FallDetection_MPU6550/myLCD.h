#ifndef MYLCD_H
#define MYLCD_H

#include <stm32f10x.h>
#include "myI2C.h"
#include <stdio.h>
#include <string.h>
#include "myDelay.h"


void LCD_I2C_Write_CMD(uint8_t data);
void LCD_I2C_Write_DATA(uint8_t data);
void LCD_I2C_Init(void);
void LCD_I2C_Clear(void);
void LCD_I2C_Location(uint8_t x, uint8_t y);
void LCD_I2C_Write_String(char* string);
void LCD_I2C_Write_Number(int number);

#endif