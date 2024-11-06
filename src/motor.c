/*
 * ECE 153B
 *
 * Name(s): Justin Wu, Nikhil Vyas
 * Section: Tuesday 7PM
 * Project
 */

#include "stm32l476xx.h"
#include "motor.h"

static const uint32_t MASK = ~GPIO_ODR_OD5 & ~GPIO_ODR_OD6 & ~GPIO_ODR_OD8 & ~GPIO_ODR_OD9;
static const uint32_t HalfStep[8] = {GPIO_ODR_OD6 | GPIO_ODR_OD8, // 0110
																		GPIO_ODR_OD6,									// 0010
																		GPIO_ODR_OD6 | GPIO_ODR_OD9,  // 0101
																		GPIO_ODR_OD9,									// 0001
																		GPIO_ODR_OD5 | GPIO_ODR_OD9,	// 1001
																		GPIO_ODR_OD5,									// 1000
																		GPIO_ODR_OD5 | GPIO_ODR_OD8,	// 1010
																		GPIO_ODR_OD8};								// 0010

static volatile int8_t dire = 0; // +1 = clockwise, -1 = counterclockwise
static volatile uint8_t step = 0;

void Motor_Init(void) {	
		// enable GPIO clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	
	// init PC5
	GPIOC->MODER &= ~(GPIO_MODER_MODE5); // reset mode bits
	GPIOC->MODER |= GPIO_MODER_MODE5_0; // set to output
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5); // clear speed bits (redundant)
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED5_1; // set speed to fast
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT5); // set to push-pull (reset bits)
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD5); // set to no PUPD (reset bits)
	
	// init PC6
	GPIOC->MODER &= ~(GPIO_MODER_MODE6); // reset mode bits
	GPIOC->MODER |= GPIO_MODER_MODE6_0; // set to output
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED6); // clear speed bits (redundant)
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED6_1; // set speed to fast
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT6); // set to push-pull (reset bits)
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD6); // set to no PUPD (reset bits)
	
	// init PC8
	GPIOC->MODER &= ~(GPIO_MODER_MODE8); // reset mode bits
	GPIOC->MODER |= GPIO_MODER_MODE8_0; // set to output
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED8); // clear speed bits (redundant)
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED8_1; // set speed to fast
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT8); // set to push-pull (reset bits)
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD8); // set to no PUPD (reset bits)
	
	// init PC9
	GPIOC->MODER &= ~(GPIO_MODER_MODE9); // reset mode bits
	GPIOC->MODER |= GPIO_MODER_MODE9_0; // set to output
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED9); // clear speed bits (redundant)
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED9_1; // set speed to fast
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT9); // set to push-pull (reset bits)
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD9); // set to no PUPD (reset bits)
}
void rotate(void) {
	step += dire;
	GPIOC->ODR &= MASK;
	GPIOC->ODR |= HalfStep[step % 8];
}

void setDire(int8_t direction) {
	dire = direction;
}
