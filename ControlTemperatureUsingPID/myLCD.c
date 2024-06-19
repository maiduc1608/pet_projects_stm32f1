#include "myLCD.h"

#define addr_pcf8574 0x27  // Address of the PCF8574 IC on the I2C bus

void I2C1_Init(void) {
    RCC->APB2ENR |= 1u<<3;  // Enable clock for Port B
    RCC->APB1ENR |= 1u<<21;  // Enable clock for I2C1

    // Configure PB6 and PB7 as I2C1 SCL and SDA pins
    GPIOB->CRL &= 0x00FFFFFF;  				// reset mode PB6,7
    GPIOB->CRL |= 0xBB000000;					// pus-pull 50MHz
    GPIOB->ODR |= ((1<<6 | 1<<7));  // Pull up I2C pins

    // Reset I2C1
    I2C1->CR1 |= 1u<<15;
    I2C1->CR1 &= ~(1u<<15);

    // Configure the frequency and clock speed for I2C1
    I2C1->CR2 |= (36<<0);  // APB1 clock frequency is 36MHz
    I2C1->CCR = 180;  // Generate 100kHz SCL frequency	// speed of transmit data
    I2C1->TRISE = 37;  // Maximum rise time

    // Enable I2C1
    I2C1->CR1 |= 1u<<0;
}

// Start I2C communication
void I2C1_Start(void) {
    I2C1->CR1 |= 1u<<8;  // Send START signal
    while ( !(I2C1->SR1 & 1u<<0) );  // Wait for START bit to be set
    (void)I2C1->SR1;  // Read SR1 to clear flag
}

// Stop I2C communication
void I2C1_Stop(void) {
    I2C1->CR1 |= 1u<<9;  // Send STOP signal
}

// Write data to I2C
void I2C1_Write(uint8_t data) {
    while ( !(I2C1->SR1 & 1u<<7) );  // Wait for TXE (Transmit Data Register Empty) to be set
    I2C1->DR = data;  // Write data to Data Register
    while ( !(I2C1->SR1 & 1u<<2) );  // Wait for BTF (Byte Transfer Finished) to be set
}

// Send address on the I2C bus
void I2C1_Send_Address(uint8_t address) {
    I2C1->DR = address << 1;  // Send slave address and write bit (0 = write)
    while (!(I2C1->SR1 & I2C_SR1_ADDR));  // Wait for ADDR to be set:
    (void)I2C1->SR1;  // Read SR1 and SR2 to clear ADDR flag
    (void)I2C1->SR2;
}

// Write a buffer of data to I2C
void I2C1_Write_Buffer(uint8_t address, uint8_t *buffer, uint8_t length) {
    I2C1_Start();
    I2C1_Send_Address(address);
    for (uint8_t i = 0; i < length; i++) {
        I2C1_Write(buffer[i]);  // Write each byte in the buffer
    }
    I2C1_Stop();
}

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