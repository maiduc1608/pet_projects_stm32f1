#ifndef MYI2C_H
#define MYI2C_H
#include<stm32f10x.h>
#include<stdlib.h>
void I2C1_Init(void);
void I2C1_Start(void);
void I2C1_Stop(void);
void I2C1_enACK(void);
void I2C1_disACK(void);
void I2C1_Write(uint8_t data);
void I2C1_Send_Address(uint8_t address);
void I2C1_Write_Buffer(uint8_t address, uint8_t *buffer, uint8_t length);
void I2C1_send1Byte(uint8_t,uint8_t,uint8_t);
void I2C1_Read1Byte(uint8_t, uint8_t*);
void I2C1_ReadMulti(uint8_t slave_address,uint8_t size, uint8_t *buffer);
//uint8_t I2C_Read(uint8_t ack);

#endif