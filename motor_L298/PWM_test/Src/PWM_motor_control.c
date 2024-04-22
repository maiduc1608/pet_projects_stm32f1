#include<stdint.h>

//configure registers
uint32_t *RCC_APB2ENR = (uint32_t*)0x40021018;
uint32_t *RCC_APB1ENR = (uint32_t*)0x4002101c; //timer3 -> PWM
uint32_t *RCC_CR = (uint32_t*)0x40021000;

//select output, input.
uint32_t *GPIOA_CRL = (uint32_t*)0x40010800;
uint32_t *GPIOA_ODR = (uint32_t*)0x4001080c;
uint32_t *GPIOA_IDR = (uint32_t*)0x40010808;

//select registers about timer(TIM3) will be use to configure PWM.
uint32_t *TIM3_CR1 = (uint32_t*)0x40000400;
uint32_t *TIM3_PSC = (uint32_t*)0x40000428;
uint32_t *TIM3_ARR = (uint32_t*)0x4000042c;
uint32_t *TIM3_CCMR1 = (uint32_t*)0x40000418;
uint32_t *TIM3_CCER = (uint32_t*)0x40000420;
uint32_t *TIM3_EGR = (uint32_t*)0x40000414;
uint32_t *TIM3_CCR1 = (uint32_t*)0x40000434;

void delay(uint32_t d){
		for(int i=0; i<d*2000;i++);
	}

void setDutyCircle(uint32_t d){
		//set duty by percent of ARR value.
		*TIM3_CCR1 = d;
	}

int main(void){
	*RCC_APB2ENR |= 0x04; //supply clock for GPIOA

	*RCC_APB1ENR |= 0x02; //supply clock for TIM3

	//configure output A6, input A0, A4.
	*GPIOA_CRL &= 0x4A464446;
	*GPIOA_CRL |= 0x4A464446;
	

	*RCC_CR |= 0x01000000; // set PLL as internal clock.(72MHz)

	//configure PWM.
	*TIM3_PSC = 72-1;	//set prescaler.
	*TIM3_ARR = 1000-1;	//set the period of auto reload register.
	*TIM3_CCMR1 &= 0x0068;
	*TIM3_CCMR1 |= 0x068;	//choose PWM mode.

	*TIM3_CR1 &= 0x01;
	*TIM3_CR1 |= 0x01; //enable counter.

	*TIM3_EGR |=0x01;
	*TIM3_CCER |= 0x01; //enable OC1 of TIM3.

	uint32_t level=0;
	uint32_t checkPress=0;
	uint32_t preCheckPress=0;
	uint32_t pinDutyValue = 0x00000001;
	uint32_t pinOnOffValue = 0x00000010;
	uint32_t checkOnOff=0;
	uint32_t preCheckOnOff=0;
	int duty[] = {0, 400, 570, 750, 900}; 	//configure 5 levels for duty circle.

	setDutyCircle(0); //initialize duty.

	while(1){
		//check: if input from PA0 is high ->turn on and change duty.
		if(((*GPIOA_IDR)&pinDutyValue)==pinDutyValue) {
			checkPress=1;
			if(preCheckPress==0) {
				level++;
				if(level==5) level=1; //set 4 levels of motor speed.
			}
		}
		else {
			checkPress = 0;
		}

		//check: if input from PA4 is high -> turn off.
		if(((*GPIOA_IDR)&pinOnOffValue)==pinOnOffValue) {
			checkOnOff=1;
			if(preCheckOnOff==0){
				level=0;
			}
		}
		else {
			checkOnOff=0;
		}
		preCheckOnOff = checkOnOff;
		preCheckPress = checkPress;
		setDutyCircle(duty[level]);
	}
}
