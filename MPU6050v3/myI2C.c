#include "myI2C.h"

#define Own_Address 0x16

void I2C1_Init(void) {
  RCC->APB1ENR |= 1<<21; // Enable I2C1 clock
	RCC->APB2ENR |= 1<<3; // en clk for GPIOB
	RCC->APB2ENR |= 0x01; //en AFIO
	GPIOB->CRL &= ~(0xFFUL<<24);
	GPIOB->CRL |= 0xFFUL<<24;
	GPIOB->ODR |= 1<<6|1<<7;

  // Set clock frequency to 36MHz
  I2C1->CR2 = 36;
  // Set own address
  I2C1->OAR1 |= Own_Address<<1 ;
	//set CCR
	I2C1->CCR &= ~(0xFFF);
	I2C1->CCR |= 1<<15; //Fast mode
	I2C1->CCR |= 0x3C;
	I2C1->TRISE = 37;  // Maximum rise time
	//ITBUFEN
	I2C1->CR2 |= 1<<10;
	//ITEVTEN
//	I2C1->CR2 |= 1<<9;
  // Enable I2C1
  I2C1->CR1 |= 1<<0;
	// Enable I2C1 acknowledge
  I2C1->CR1 |= 1<<10;
}

// Start I2C communication
void I2C1_Start(void) {
    I2C1->CR1 |= 1u<<8;  // Send START signal
    while ( !(I2C1->SR1 & 1u<<0) );  // Wait for START bit to be set
 //   (void)I2C1->SR1;  // Read SR1 to clear flag
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
    I2C1->DR = address;  // Send slave address and write bit (0 = write)
    while (!(I2C1->SR1 & 1<<1));  // Wait for ADDR to be set:
    (void)I2C1->SR1;  // Read SR1 and SR2 to clear ADDR flag
    (void)I2C1->SR2;
}

// Write a buffer of data to I2C
void I2C1_Write_Buffer(uint8_t address, uint8_t *buffer, uint8_t length) {
    I2C1_Start();
    I2C1_Send_Address(address<<1); //Send addr with write bit
    for (uint8_t i = 0; i < length; i++) {
        I2C1_Write(buffer[i]);  // Write each byte in the buffer
    }
    I2C1_Stop();
}

//uint8_t I2C_Read(uint8_t ack){
//    
//    while(!(I2C1->SR1 & I2C_SR1_RXNE)); // Wait until data is received
//    return I2C1->DR; // Read data
//}

void I2C1_enACK(void){
	I2C1->CR1 |= 1<<10;
}

void I2C1_disACK(void){
	I2C1->CR1 &= ~(1u<<10);
}

void I2C1_send1Byte(uint8_t slave_address, uint8_t reg_address, uint8_t data){
	I2C1_Start();
	I2C1_Send_Address(slave_address<<1);
	I2C1_Write(reg_address);
	I2C1_Write(data);
	I2C1_Stop();
}

void I2C1_Read1Byte(uint8_t slave_address, uint8_t *data){
	I2C1->CR1 |= 1<<10; //en ack
	//1. send start
	I2C1_Start();
	//2. send salve address
	I2C1->DR = (slave_address<<1)|1;
	while (!(I2C1->SR1 & 1<<1));  // Wait for ADDR to be set:
	//3. clear ack
	I2C1->CR1 &= ~(1UL<<10);
	//4. clear ADDR
	I2C1->SR1;
	I2C1->SR2;
	//5. set stop
	I2C1->CR1 |= 1<<9;
	//6. wait RxNE and read data
	while(!(I2C1->SR1 & 1<<6));
	*data = I2C1->DR;
	
}

void I2C1_ReadMulti(uint8_t slave_address,uint8_t size, uint8_t *buffer){
//	buffer = (char)malloc(size * sizeof(char));
	I2C1_enACK();
	//1. send start
	I2C1_Start();
	//2. send salve address 
	I2C1->DR = (slave_address<<1)|1;
	while (!(I2C1->SR1 & 1<<1));  // Wait for ADDR to be set:
	//3. clear ADDR
	I2C1->SR1;
	I2C1->SR2;
	//4. read to data n-1
	uint8_t i;
	for(i=0;i<size-1;i++){
		while(!(I2C1->SR1 & 1<<6));
		buffer[i] = I2C1->DR;
	}
	//5. clear ack and set stop
	I2C1_disACK();
	I2C1_Stop();
	//6. read last data
	while(!(I2C1->SR1 & 1<<6));
	buffer[i] = I2C1->DR;
}