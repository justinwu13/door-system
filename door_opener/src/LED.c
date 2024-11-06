/*
 * ECE 153B
 *
 * Name(s): Justin Wu, Nikhil Vyas
 * Section: Tuesday 7PM
 * Project
 */

#include "LED.h"

void LED_Init(void) {
	// Enable GPIO Clocks
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	
	// Initialize Green LED
	GPIOA->MODER &= ~GPIO_MODER_MODE5; // Clear mode bits 
	GPIOA->MODER |= GPIO_MODER_MODE5_0; // set A5 to output
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT5; // clear output type bits (set to push-pull)
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD5; // set to no pull up/pull down
}

void LED_Off(void) {
	GPIOA->ODR &= ~GPIO_ODR_OD5;
}

void LED_On(void) {
	GPIOA->ODR |= GPIO_ODR_OD5;
}

void LED_Toggle(void) {
	GPIOA->ODR ^= GPIO_ODR_OD5;
}
