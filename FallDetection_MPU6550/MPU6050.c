#include <stm32f10x.h>
#include <math.h>
#include <stdbool.h>
#include "mySystemClockConfig.h"
#include "myI2C.h"
#include "myLCD.h"
#include "myDelay.h"
#include "myTimer.h"

#define RedLedPin 1ul<<2;
#define GreenLedPin 1ul<<13;
#define MPU6050_ADDRESS 0x68
#define acc_xh_addr 0x3B
#define gyro_xh_addr 0x43
#define sample_rate 25
#define config 26
#define gyro_config 27
#define acc_config 28
#define Level_INT 55
#define interrupts 56
#define pwr_manager 107
#define accel_sensitivity 16384.0
#define gyro_sensitivity 65.5
#define SampleRate (uint8_t)5

#define FALL_THRESHOLD_LOW 25.0
#define FALL_THRESHOLD_HIGH 150.0
#define GYRO_THRESHOLD 700.0    

enum STATE{
	NORMAL,
	FALLED,
	STOP
};

enum STATE State = NORMAL;

void EXTI_config(void);
void MPU_config(void);
void MPU_Init(void);
void MPU_read_register(uint8_t, uint8_t*);
void MPU_read_multi(uint8_t reg_address,uint8_t size, uint8_t *buffer);
void MPU_Display(void);
void LCD_Display(void);
void LED_Init(void);
void Enter_Stop_Mode(void);
void EXTI2_IRQHandler(void);
void TIMER_config(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void Interrupt_Manager(void);
int detectFall();

static uint8_t acc_buffer[6];
static uint8_t gyro_buffer[6];
static float ax;
static float ay;
static float az;
static float gx;
static float gy;
static float gz;
static float total_acceleration;
static float total_gyro;

static int system_started = 0;
static int fall_detected = 0;
static int in_free_fall = 0;
static int check_collide1 = 0;
static int check_collide2 = 0;
static int tick_count = 0;

int main(void){
  SysClkConf_72MHz();
  LED_Init();
	TIM2_Setup(7200,5000); //Red
	TIM3_Setup(7200,10000); //Green
	TIM3_GreenState(1);
  I2C1_Init();
	LCD_I2C_Init();
	LCD_I2C_Clear();
	LCD_Display();
  MPU_Init();
	Delay_ms(50);
	MPU_Init();
	Delay_ms(50);
	EXTI_config();
	system_started = 1;
	Interrupt_Manager();
  while(1){
		if(State == STOP){
			LCD_I2C_Location(1,8);
			LCD_I2C_Write_String("STOP  ");
			Enter_Stop_Mode();
			State = NORMAL;
		}
		if(State == NORMAL){
			fall_detected = 0;
		}
		else if(State == FALLED) {
			TIM2_RedState(1);
			while(1){
				if(State == STOP){
					TIM2_RedState(0);
					break;
				}
			}
		}
	}
}

void LED_Init(void){
	//enable clock for GPIOA, GPIOC
  RCC->APB2ENR |= 1<<4 | 1<<2; 
  GPIOC->CRH &= ~(0xFUL<<20);
  GPIOC->CRH |= 0x3<<20; //C13 output pp
	GPIOA->CRL &= ~(0xFul<<8);
	GPIOA->CRL |= 0x3<<8; //A2 output pp
}

void EXTI_config(void){
	RCC->APB2ENR |= 1<<0 | 1<<2; //en AFIO and GPIOA
	GPIOA->CRL &= ~(0xFFUL);
	GPIOA->CRL |= 0x48;
	GPIOA->ODR &= ~(1<<0);
	AFIO->EXTICR[0] &= ~(0xFUL<<0); //EXTI A0
	AFIO->EXTICR[0] &= ~(0xFUL<<4); //EXTI A1
	EXTI->IMR |= 1<<0|1<<1;
	EXTI->EMR = 0;
	EXTI->RTSR &= ~(1UL<<1 | 1ul<<0);
	EXTI->FTSR |= 1<<1 | 1<<0;  //falling trigger selected
}

void Enter_Stop_Mode(void) {
  NVIC_DisableIRQ(EXTI1_IRQn);
  RCC->APB1ENR |= 1<<28; //en PWR
	// Clear Wake-up flag
  PWR->CR |= PWR_CR_CWUF;
	PWR->CSR|= 1<<8;
  // Set SLEEPDEEP bit of Cortex System Control Register
  SCB->SCR |= 1<<2;
  // Clear PDDS bit in PWR_CR to select Stop mode
  PWR->CR &= ~(1UL<<1);
  // Set LPDS bit in PWR_CR to select low-power mode
  PWR->CR |= 1<<0;
	// Enter Stop mode
  __WFI();
  SysClkConf_72MHz();
  NVIC_EnableIRQ(EXTI1_IRQn);
}

void EXTI0_IRQHandler(void){
    if(EXTI->PR & 1<<0){
			if(State != STOP) State = STOP;
      EXTI->PR |= 1<<0;
    }
}

void EXTI1_IRQHandler(void){
  if(EXTI->PR & 1<<1){
		MPU_read_multi(acc_xh_addr,6,acc_buffer);
		MPU_read_multi(gyro_xh_addr,6,gyro_buffer);
    int16_t ax_raw = (int16_t)(acc_buffer[0] << 8 | acc_buffer[1]);
    int16_t ay_raw = (int16_t)(acc_buffer[2] << 8 | acc_buffer[3]);
    int16_t az_raw = (int16_t)(acc_buffer[4] << 8 | acc_buffer[5]);
    ax = (float)ax_raw / accel_sensitivity;
    ay = (float)ay_raw / accel_sensitivity;
    az = (float)az_raw / accel_sensitivity;
		az -= 0.1;
		
    int16_t gx_raw = (int16_t)(gyro_buffer[0] << 8 | gyro_buffer[1]);
    int16_t gy_raw = (int16_t)(gyro_buffer[2] << 8 | gyro_buffer[3]);
    int16_t gz_raw = (int16_t)(gyro_buffer[4] << 8 | gyro_buffer[5]);
    gx = (float)gx_raw / gyro_sensitivity;
    gy = (float)gy_raw / gyro_sensitivity;
    gz = (float)gz_raw / gyro_sensitivity;

    total_acceleration = sqrt(ax * ax + ay * ay + az * az) * 100;
    total_gyro = sqrt(gx * gx + gy * gy + gz * gz);
		//MPU_Display();
		
		if(State != FALLED){
			LCD_I2C_Location(0,8);
			LCD_I2C_Write_String("0");
			LCD_I2C_Location(1,8);
			LCD_I2C_Write_String("NORMAL");
			if(detectFall()) {

				if(State != STOP) State = FALLED;
				fall_detected = 300;
			} 
			
			if (in_free_fall) tick_count += SampleRate;
			if (!detectFall() && tick_count > 600) {
				tick_count = 0;
				in_free_fall = 0;
				check_collide1 = 0;
				check_collide2 = 0;
			}
		}
		else {
			LCD_I2C_Location(0,8);
			LCD_I2C_Write_String("1");
			LCD_I2C_Location(1,8);
			LCD_I2C_Write_String("FALLED");
		}
	}
	//clear pending bit
	EXTI->PR |= 1 << 1;
}

int detectFall() {
	if (!system_started) return 0;   // Skip if system is not fully started
	if (total_acceleration <= FALL_THRESHOLD_LOW) {
		in_free_fall = 100; // Device is in free fall
	}
	
	if (in_free_fall) {
		if (total_acceleration > FALL_THRESHOLD_HIGH) {
			check_collide1 = 150;
		}
		if (total_gyro > GYRO_THRESHOLD) {
			check_collide2 = 200;
		}
		if (check_collide1 && check_collide2) {
			in_free_fall = 0;
			check_collide1 = 0;
			check_collide2 = 0;
			return 1;  // Fall detected based on both acceleration and gyro rate
		}
	}
	return 0;
}

void MPU_read_register(uint8_t reg_address, uint8_t *data){
	//1. start
	I2C1_Start();
	//2. send slave address with write bit
	I2C1_Send_Address(MPU6050_ADDRESS<<1);
	//3. send MPU internal register address
	I2C1_Write(reg_address);
	//4. read 1 byte data
	I2C1_Read1Byte(MPU6050_ADDRESS, data);
}

void MPU_read_multi(uint8_t reg_address,uint8_t size, uint8_t *buffer){
	//1. start
	I2C1_Start();
	//2. send slave address with write bit
	I2C1_Send_Address(MPU6050_ADDRESS<<1);
	//3. send MPU internal register address
	I2C1_Write(reg_address);
	I2C1_ReadMulti(MPU6050_ADDRESS,6,buffer);
	I2C1_Stop();
}

void MPU_Init(void){
	I2C1_send1Byte(MPU6050_ADDRESS, sample_rate,SampleRate - 1);
  I2C1_send1Byte(MPU6050_ADDRESS, config, 0x29); //FSYNC = acc_x, DLPF = 1
  I2C1_send1Byte(MPU6050_ADDRESS, gyro_config, 0x10); // ±1000°/s
  I2C1_send1Byte(MPU6050_ADDRESS, acc_config, 0x00);
  I2C1_send1Byte(MPU6050_ADDRESS, Level_INT, 0x80);
  I2C1_send1Byte(MPU6050_ADDRESS, interrupts, 1u);
  I2C1_send1Byte(MPU6050_ADDRESS, pwr_manager, 1u);
}

void TIM2_IRQHandler(void){
	if(TIM2->SR & 1UL<<0){
		GPIOA->ODR ^= 1<<2;
		TIM2->SR &= ~(1UL<<0); //clear interrupt flag
	}
}

void TIM3_IRQHandler(void){
	if(TIM3->SR & 1UL<<0){
		GPIOC->ODR ^= 1<<13;
		TIM3->SR &= ~(1UL<<0);
	}
}

void LCD_Display(void){
	LCD_I2C_Location(0,0);
	LCD_I2C_Write_String("FALLED?: ");
	LCD_I2C_Location(1,0);
	LCD_I2C_Write_String("STATUS: ");
}

void Interrupt_Manager(void){
	NVIC_SetPriority(TIM2_IRQn,2);
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM3_IRQn,2);
	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority(EXTI1_IRQn,1);
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_SetPriority(EXTI0_IRQn,0);
	NVIC_EnableIRQ(EXTI0_IRQn);
}