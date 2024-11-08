/*
 * ECE 153B
 *
 * Name(s): Justin Wu, Nikhil Vyas
 * Section: Tuesday 7PM
 * Project
 */

#ifndef __STM32L476G_NUCLEO_SYSTICK_H
#define __STM32L476G_NUCLEO_SYSTICK_H

#include "stm32l476xx.h"

void SysTick_Init(void);
void SysTick_Handler(void);
void delay(uint32_t ms);
void delay3s(void);

#endif
