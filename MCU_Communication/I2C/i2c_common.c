#include<stdint.h>
#include<stm32f10x.h>

#define Own_Address 0x16
#define Slave_Address 0x03

void SysClkConf(void);
void PinConf(void);
void I2C1_Init(void);
void I2C1_Start(void);
void I2C1_Stop(void);
void I2C1_SendData(uint8_t data);
void I2C1_SendAddr(uint8_t addr);

void EXTI_Conf(void);
void EXTI1_IRQHandler(void);S

static uint32_t i2c_DR = 0;
static uint32_t i2c_CR1;
static uint32_t i2c_CR2;
static uint32_t i2c_OAR1;
static uint32_t i2c_CCR;

int main(void){
	SysClkConf();
	PinConf();
	I2C1_Init();
	EXTI_Conf();
	
	while(1){
		i2c_DR = I2C1->DR;
		i2c_CCR = I2C1->CCR;
		i2c_CR1=I2C1->CR1;
		i2c_CR2=I2C1->CR2;
		i2c_OAR1=I2C1->OAR1;
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
}

void PinConf(void){
	RCC->APB2ENR |= 1<<4;
	GPIOC->CRH &= ~(0xF<<20);
	GPIOC->CRH |= 0x3<<20; // C13
}

void I2C1_Init(void){
	RCC->APB1ENR |= 1<<21; // Enable I2C1 clock
	RCC->APB2ENR |= 1<<3; // en clk for GPIOB
	GPIOB->CRL &= ~(0xFF<<24);
	GPIOB->CRL |= 0xFF<<24;
	RCC->APB2ENR |= 0x01; //en AFIO

  // Set clock frequency to 36MHz
  I2C1->CR2 = 36;
  // Set own address
  I2C1->OAR1 |= Own_Address<<1 ;
	//set CCR
	I2C1->CCR &= ~(0xFFF);
	I2C1->CCR |= 0xB4;
	//ITEVTEN & ITBUFEN
	I2C1->CR2 |= 1<<10 | 1<<9;
  // Enable I2C1
  I2C1->CR1 |= 1<<0;
	// Enable I2C1 acknowledge
  I2C1->CR1 |= 1<<10;
  // Enable I2C1 interrupt
	NVIC_SetPriority(I2C1_EV_IRQn,2);
	NVIC_EnableIRQ(I2C1_EV_IRQn);
}

void I2C1_Start(void)
{
    I2C1->CR1 |= 1<<8;
    while (!(I2C1->SR1 & 1<<0)); // Wait for start bit sent
}

void I2C1_Stop(void)
{
    I2C1->CR1 |= 1<<9;
}

void I2C1_SendAddr(uint8_t addr)
{
  I2C1->DR = addr;
	while (!(I2C1->SR1 & 1<<1)); // Wait for address sent
  I2C1->SR2; // Clear ADDR flag
}

void I2C1_SendData(uint8_t data)
{
	while(!(I2C1->SR1 & 1<<7)); //wait TxE: data register empty
  I2C1->DR = data;
	while (!(I2C1->SR1 & 1<<2)); // Wait for data sent
	
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
//		GPIOC->ODR ^= 1<<13;
		I2C1_Start();	
    I2C1_SendAddr(Slave_Address<<1); // Send slave address for write
    I2C1_SendData(0xAA); // Send data
    I2C1_Stop();
		//clear IRQ flag
		EXTI->PR |= 1<<1;
	}
}

void I2C1_EV_IRQHandler(void)
{
	//handle interrupt when ADDR bit = 1.
  if (I2C1->SR1 & 1<<1){
		I2C1->SR2; // Clear ADDR flag after send or receive address byte.
  }
	//handle interrupt when RXNE bit = 1.
	if (I2C1->SR1 & 1<<6){
		uint8_t receivedData = I2C1->DR;
    if(receivedData==0xAA){
		GPIOC->ODR ^= 1<<13;
		}
  }
}
