#include "myLCD.h"

#define addr_pcf8574 0x27  // Address of the PCF8574 IC on the I2C bus

// Send command to LCD via I2C
void LCD_I2C_Write_CMD(uint8_t data) {
    uint8_t buf[4] = {
        (data & 0xF0) | 0x04,  // Send high nibble with E=1
        (data & 0xF0),         // Send high nibble with E=0
        (data << 4) | 0x04,    // Send low nibble with E=1
        (data << 4)            // Send low nibble with E=0
    };
    I2C1_Write_Buffer(addr_pcf8574, buf, 4);  // Send buffer to PCF8574
}

// Send data to LCD via I2C
void LCD_I2C_Write_DATA(uint8_t data) {
    uint8_t buf[4] = {
        (data & 0xF0) | 0x05,  // Send high nibble with E=1 and RS=1
        (data & 0xF0) | 0x01,  // Send high nibble with E=0 and RS=1
        (data << 4) | 0x05,    // Send low nibble with E=1 and RS=1
        (data << 4) | 0x01     // Send low nibble with E=0 and RS=1
    };
    I2C1_Write_Buffer(addr_pcf8574, buf, 4);  // Send buffer to PCF8574
}

// Initialize LCD via I2C
void LCD_I2C_Init(void) {
    Delay_ms(50);  		
    LCD_I2C_Write_CMD(0x33);  // Initialization
    LCD_I2C_Write_CMD(0x32);  // Initialization
    LCD_I2C_Write_CMD(0x28);  // 4-bit mode, 2 lines, 5x7 font // function set
    LCD_I2C_Write_CMD(0x0C);  // Display ON, cursor OFF
    LCD_I2C_Write_CMD(0x06);  // Increment cursor
    LCD_I2C_Write_CMD(0x01);  // Clear display
    Delay_ms(2);
}

// Clear the LCD display
void LCD_I2C_Clear(void) {
    LCD_I2C_Write_CMD(0x01);  // Send clear display command
    Delay_ms(2);  // Wait for command to execute
}

// Set cursor position on LCD
void LCD_I2C_Location(uint8_t x, uint8_t y) {
    if (x == 0) {
        LCD_I2C_Write_CMD(0x80 + y);  // Set position to line 0
    } else if (x == 1) {
        LCD_I2C_Write_CMD(0xC0 + y);  // Set position to line 1
    }
}

// Write a string to LCD
void LCD_I2C_Write_String(char* string) {
    for (uint8_t i = 0; i < strlen(string); i++) {
        LCD_I2C_Write_DATA(string[i]);  // Send each character
    }
}

// Write a number to LCD
void LCD_I2C_Write_Number(int number) {
    char buffer[8];
    sprintf(buffer, "%d", number);  // Convert number to string
    LCD_I2C_Write_String(buffer);  // Send string to LCD
}