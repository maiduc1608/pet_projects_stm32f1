#include<stdint.h>
#include<math.h>
#include<stm32f10x.h>

#define PI 3.141592653589793
#define TABLE_SIZE 255     // size of lookup table
#define AMPLITUDE 255      // amplitude of pulse.(8bit)

void SysClkConf(void);
void PinConf(void);

void delay_ms(uint32_t);

void ADC_conf(void);
void ADC_start(void);
void ADC1_2_IRQHandler(void);

void SineTable_Init(void);
void TriangleTable_Init(void);
void SquareTable_Init(void);

void Timer3_DAC_Config(void);
void TIM3_IRQHandler(void);
void EXTI_Config(void);
void EXTI15_10_IRQHandler(void);
void reset_pusle_index(void);

void UART_Conf(void);
void transmit_uart(uint8_t);

static uint32_t vol ;
static uint16_t DR_value;
static uint32_t CR_value;
static uint32_t CFGR_value;
static uint32_t ODR_value = 0;
static uint32_t choice=0;
static uint32_t freq_choice=0;
static uint32_t freq_choice_changed=0;
static uint32_t freq_table[3] = {60000,30000,5000};// levels of TIM3_ARR.
static uint8_t data_uart;

static uint16_t sine_table[TABLE_SIZE];
static uint16_t triangle_table[TABLE_SIZE];
static uint16_t square_table[TABLE_SIZE];

// index variables for each lookup table.
static uint16_t triangle_index = 0;
static uint16_t sine_index = 0;
static uint16_t square_index = 0;

int main(void){
	SineTable_Init();
	TriangleTable_Init();
	SquareTable_Init();

	SysClkConf();
	PinConf();
	UART_Conf();
	Timer3_DAC_Config();
	EXTI_Config();

	ADC_conf();
	ADC_start();
	while(1){
		transmit_uart(data_uart);
		vol = (uint32_t)(ADC1->DR * (43.1/65536)); // change to 4,3V
		CR_value = RCC->CR;
		CFGR_value = RCC->CFGR;
		ODR_value = GPIOB->ODR;
		 if (freq_choice_changed) {
            TIM3->ARR = freq_table[freq_choice]-1;
            freq_choice_changed = 0;
        }
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
	RCC->APB2ENR |= 1<<9; // en clk for ADC1.
	RCC->APB2ENR |= 0x01; // en clk for AFIO.
	RCC->APB1ENR |= 0x03; // en clk for TIM2, 3.
	RCC->APB2ENR |= 1<<14 | 1<<2; // en clk for usart1, gpio A
}

void PinConf(){
	//ADC
	GPIOA->CRL &= 0x44444400; // mode A0, A1 analog.
	//GPIOC13 output
	GPIOC->CRH &= ~(0xF<<20);
	GPIOC->CRH |= 0x02<<20;
	GPIOC->ODR &= ~(1<<13);
	//DAC
	GPIOB->CRH = 0x33333333; // output pp 50MHz.
	//EXTI_A11, A12:
	GPIOA->CRH &= ~(0xFF<<12);
	GPIOA->CRH |= 1<<15 | 1<<19;
	//USART A9, A10
	GPIOA->CRH &= ~(0xFF<<4);
	GPIOA->CRH |= 0xBB<<4;
 }

void delay_ms(uint32_t milliseconds) {
    uint32_t i;
    for(i = 0; i < milliseconds; i++) {
        uint32_t delay_count = 4000;
        while(delay_count--);
    }
}

void ADC_conf(void){

	ADC1->CR2 &= ~(1<<11); // align
	ADC1->CR2 |= 0x02; // continuos
	ADC1->SMPR2 |= 0x03; // sampling
	ADC1->SQR3 = 0x00; //A0.
	ADC1->CR1 |= 1<<5;
	ADC1->CR2 |= 0x01;// ADON to power up adc.
	NVIC_SetPriority(ADC1_2_IRQn, 0); // set priority for interrupt
  NVIC_EnableIRQ(ADC1_2_IRQn); // enable interrupt for ADC1
}
void ADC_start(void){
	delay_ms(100);
	ADC1->CR2 |= 0x01; // ADON to launch
	delay_ms(100);
	ADC1->CR2 |= 1<<22; // start.
	delay_ms(10);
}

void ADC1_2_IRQHandler(void) {
    // check whether ADC interupt has been enabled or not.
    if ((ADC1->SR & 0x02) != 0) { // End of Conversion flag
			DR_value = (uint16_t)ADC1->DR; // read data from ADC1_DR register.
			data_uart = (uint8_t) (DR_value * (255.0/4095.0));
      ADC1->SR &= ~(0x02); // Clear EOC flag
    }
}

void SineTable_Init(void) {
    // create table lookup for sine pulse
    for (int i = 0; i < TABLE_SIZE; i++) {
        sine_table[i] = (uint8_t)((AMPLITUDE/2) * (1 + sin(2*PI*i/TABLE_SIZE)));
			sine_table[i]=sine_table[i]<<8;
    }
}

void TriangleTable_Init(void) {
    // create table lookup
    for (int i = 0; i < TABLE_SIZE /2; i++) {
        triangle_table[i] = (uint8_t)(i*2); 
			triangle_table[i]=triangle_table[i]<<8;
			if(triangle_table[i]==256) triangle_table[i]=255;
    }
    for (int i = TABLE_SIZE /2; i < TABLE_SIZE; i++) {
        triangle_table[i] = (uint8_t)(AMPLITUDE - i)*2;
			triangle_table[i]=triangle_table[i]<<8;
		}
}

void SquareTable_Init(void) {
    // create table lookup for square pulse
    for (int i = 0; i < TABLE_SIZE / 2; i++) {
				square_table[i] = 0x0;
    }
    for (int i = TABLE_SIZE / 2; i < TABLE_SIZE; i++) {
        square_table[i] = 0xFF00;
    }
}

// config timer to control DAC.
void Timer3_DAC_Config(void) {
    // enable clock for timer3
    RCC->APB1ENR |= 1<<7;

    // Config TIM3
    TIM3->PSC = 9 - 1;		// Prescaler: 72MHz to 8MHz
    TIM3->ARR = freq_table[freq_choice];   // initialize TIM_ARR
    TIM3->DIER |= 0x01;
	TIM3->CR1 |= TIM_CR1_CEN;  // enable timer
	NVIC_SetPriority (TIM3_IRQn,2);  // Set Priority for interrupt
	NVIC_EnableIRQ(TIM3_IRQn); // enable interrupt TIM3
}

//Config interrupt for TIM3
void TIM3_IRQHandler(void){
    // check interrupt flag.
    if ((TIM3->SR & 0x01) == 0x01) {
        // clear interrupt flag.
        TIM3->SR &= ~(0x01);
        // change the type of pulse.
      switch (choice){
        	case 1:
						GPIOB->ODR = triangle_table[triangle_index];
						// Tang ch? m?c c?a b?ng lookup
						triangle_index++;
						if (triangle_index >= TABLE_SIZE) {
							triangle_index = 0;
						}
        		break;
        	case 0:
						GPIOB->ODR = sine_table[sine_index];
						// Tang ch? m?c c?a b?ng lookup
						sine_index++;
						if (sine_index >= TABLE_SIZE) {
							sine_index = 0;
						}
        		break;
					case 2:
						GPIOB->ODR = square_table[square_index];
						// Tang ch? m?c c?a b?ng lookup
						square_index++;
						if (square_index >= TABLE_SIZE) {
							square_index = 0;
						}
        		break;
        }
    }
}

// Config external interrupts to choose type of pulse or change the frequency
void EXTI_Config(void){
	AFIO->EXTICR[3] &= ~(0xF<<12); // EXTI A15
	AFIO->EXTICR[2] &= ~(0xF<<12); //EXTI A11
	//config for EXTI A11
	EXTI->IMR |= 1<<11;
	EXTI->FTSR &= ~(1<<11);
	EXTI->RTSR |= 1<<11;
	//config for EXTI A15
	EXTI->IMR |= 1<<15;
	EXTI->FTSR &= ~(1<<15);
	EXTI->RTSR |= 1<<15;
	NVIC_SetPriority (EXTI15_10_IRQn, 1);  // Set Priority
	NVIC_EnableIRQ (EXTI15_10_IRQn);  // Enable Interrupt
}

void EXTI15_10_IRQHandler(void) {
	//1. control external interrupt at GPIO A11
    // check the interrupt flag
    if (EXTI->PR & 1<<11) {
        // handle when interrupt occur at A11
			choice++;
			if (choice>2) choice = 0;
			reset_pusle_index();
			GPIOC->ODR ^= 1<<13; // toggle LED at C13 to check.
			//clear the flag.
			EXTI->PR |= 1<<11;
    }
    //2. control external interrupt at GPIO A13
    //check the interrupt flag
		if (EXTI->PR & 1<<15) {
        // handle when interrupt occur at A13
			freq_choice++;
			if (freq_choice>2) freq_choice = 0;
			freq_choice_changed = 1;
			reset_pusle_index();
			GPIOC->ODR ^= 1<<13;
			// clear the flag.
			EXTI->PR |= 1<<15;
    }
}

void reset_pusle_index(void){
	square_index &= 0;
	sine_index &= 0;
	triangle_index &= 0;
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
	//5.set TE bit
	USART1->CR1 |= 1<<3;	
}
void transmit_uart(uint8_t data) {
    // Wait until the transmit data register is empty (TXE flag)
    while (!(USART1->SR & 1<<7)) {}
    USART1->DR = data;
    // Wait until the character is transmitted (TC flag)
    while (!(USART1->SR & 1<<6)) {}
}
