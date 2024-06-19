#include "myUART.h"

void myUART_Config(void){
	RCC->APB2ENR |= 1<<14 | 1<<2; // en clk for usart1, gpio A
	GPIOA->CRH &= ~(0xFFUL<<4);
	GPIOA->CRH |= 0x8B<<4;
	//1. enable USART
	USART1->CR1 |= 1<<13;
	//2. define word length.
	USART1->CR1 &= ~(1<<12); // 8bit
	//3. stop bits
	USART1->CR2 &= ~(3<<12); //1 stop bit
	//5. Baud rate
	USART1->BRR |= 0x1D4C; //9600
	//6.set TE and RE bit
	USART1->CR1 |= 1<<3|1<<2;
	//set RXNEIE interrupt
	USART1->CR1 |= 1<<5;
	NVIC_SetPriority(USART1_IRQn,0);
	NVIC_EnableIRQ(USART1_IRQn);
}
void uart_transmit_data(char *data) {
  int i = 0;
  // Loop through each character in the message and transmit
  while (data[i] != 'e') {
    // Wait until the transmit data register is empty (TXE flag)
    while (!(USART1->SR & USART_SR_TXE)) {}
    USART1->DR = data[i++];
    // Wait until the character is transmitted (TC flag)
    while (!(USART1->SR & USART_SR_TC)) {}
  }
}

