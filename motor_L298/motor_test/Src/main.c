#include <stdint.h>

uint32_t *RCC_APB2ENR = (uint32_t*)0x40021018;
uint32_t *GPIOA_CRL = (uint32_t*)0x40010800;

uint32_t *TIM2_ARR = (uint32_t*)0x4000002C;
uint32_t *TIM2_CCR2 = (uint32_t*)0x40000038;
uint32_t *TIM2_CNT = (uint32_t*)0x40000024;
uint32_t *TIM2_PSC = (uint32_t*)0x40000028;
uint32_t *TIM2_CR1 = (uint32_t*)0x40000000;
uint32_t *TIM2_CCER = (uint32_t*)0x40000020;
uint32_t *TIM2_CCMR1 = (uint32_t*)0x40000018;

void setDutyCycle(uint16_t dutyCycle) {
    *TIM2_CCR2 = dutyCycle;
}

void delay(int d ) {
    for (volatile uint32_t j = 0; j < d*2000; j++) {
        // Delay
    }
}
int main(void) {
    // Enable clock for GPIOA
    *RCC_APB2ENR |= 0x04;

    // Configure PA1 to alternate function output push-pull (Mode: 10)
    *GPIOA_CRL &= 0xFFFFF0FF; // Clear bits 12-15
    *GPIOA_CRL |= 0x00000020; // Set bits 9-10 for mode 10

    // Configure PWM
    *TIM2_ARR = 1000 - 1;  // Period of PWM signal
    *TIM2_PSC = 72 - 1;     // Prescaler
    *TIM2_CCER |= 0x00000010;  // Enable capture/compare 2 output
    *TIM2_CCMR1 |= 0x0060;     // PWM mode 1, preload enable
    *TIM2_CR1 |= 0x01;         // Enable TIM2

    setDutyCycle(500);  // Set duty cycle to 50%

    while (1) {
        // Your application code here
    }
}
