#include<stdint.h>
#include<math.h>
#include<stm32f10x.h>

#define PI 3.141592653589793
#define TABLE_SIZE 255     // size of lookup table
#define AMPLITUDE 255      // amplitude of pulse.(8bit)

void SysClkConf(void);
void PinConf(void);

void delay_ms(uint32_t);

void UART_Conf(void);
void USART1_IRQHandler(void);
void transmit_uart(uint8_t);

void EXTI_Conf(void);
	
static uint8_t transmit_data;
static uint8_t received_data;


int main(void){

	SysClkConf();
	PinConf();
	UART_Conf();
	EXTI_Conf();

	while(1){
	}
}

void SysClkConf(void) {

	//1.Turn on HSE to use.
	RCC->CR |= 1<<16; // HSE on
	while((RCC->CR & (1<<17)) == 0); // wait HSERDY.

	//2. config PLL (HSE, MUL9).
	RCC->CFGR |= 0x07<<18; // PLLMUL9 -> systemclock = 72MHz
	RCC->CFGR |= 1<<15; // ADC prescale 6.
	RCC->CFGR |= 1<<10; //APB1 prescale 2.

	//3. choose new clock source.
	RCC->CFGR |= (1<<16); // PLLSRC HSE

	//4. enable PLL
	RCC->CR |= 1<<24;
	while((RCC->CR & 1<<25) == 0); // wait PLLRDY.

	//5. switch sysclk to PLL
	RCC->CFGR |= 0x02;
	while((RCC->CFGR & (1<<3)) == 0); //wait SWS.

	//6.turn off original source
	RCC->CR &= ~(0x01); // off HSION
	while((RCC->CR & 0x02)==0x02);
	//7. supply clock
	RCC->APB2ENR |= 0x7<<2; //en clk for GPIOA, B, C.
	RCC->APB2ENR |= 0x01; // en clk for AFIO.
	RCC->APB2ENR |= 1<<14; // en clk for usart1.
}

void PinConf(){
	//GPIOC13 output
	GPIOC->CRH &= ~(0xF<<20);
	GPIOC->CRH |= 0x02<<20;
	GPIOC->ODR &= ~(1<<13);
	//USART A9, A10
	GPIOA->CRH &= ~(0xFF<<4);
	GPIOA->CRH |= 0x8B<<4;
 }

void delay_ms(uint32_t milliseconds) {
    uint32_t i;
    for(i = 0; i < milliseconds; i++) {
        uint32_t delay_count = 4000;
        while(delay_count--);
    }
}

void UART_Conf(void){
	//1. enable USART
	USART1->CR1 |= 1<<13;
	//2. define word length.
	USART1->CR1 &= ~(1<<12); // 8 bits length
	//3. stop bits
	USART1->CR2 &= 1<<13; //2 stop bits
	//4. Baud rate
	USART1->BRR |= 0x1D4C; // baud rate = 9600
	//5.set TE and TXEIE bit
	USART1->CR1 |= 1<<3;	
	//6.set RE and RXNEIE bit
	USART1->CR1 |= 1<<2 | 1<<5;
	NVIC_EnableIRQ(USART1_IRQn); //enable interrupt for uart
}
void transmit_uart(uint8_t data) {
    // Wait until the transmit data register is empty (TXE flag)
    while (!(USART1->SR & 1<<7)) {}
    USART1->DR = data;
    // Wait until the character is transmitted (TC flag)
    while (!(USART1->SR & 1<<6)) {}
}
void USART1_IRQHandler(void){
	if((USART1->SR & 1<<5) == 1<<5){
		received_data = (uint8_t)USART1->DR;
		if(received_data == 0xAC) GPIOC->ODR ^= 1<<13;
		USART1->SR &= ~(1<<5);
	}
}

void EXTI_Conf(void){
	//A1
	RCC->APB2ENR |= 1<<2;
	GPIOA->CRL &= ~(0xF<<4);
	GPIOA->CRL |= 1<<6;
	
	AFIO->EXTICR[0] &= ~(0xF<<4); // EXTI A1
	//config for EXTI A1
	EXTI->IMR |= 1<<1;
	EXTI->FTSR &= ~(1<<1);
	EXTI->RTSR |= 1<<1;
	NVIC_SetPriority (EXTI1_IRQn, 1);  // Set Priority
	NVIC_EnableIRQ (EXTI1_IRQn);  // Enable Interrupt
}

void EXTI1_IRQHandler(void){
	if(EXTI->PR & 1<<1){
		transmit_uart(0xAC);
		//clear IRQ flag
		EXTI->PR |= 1<<1;
	}
}