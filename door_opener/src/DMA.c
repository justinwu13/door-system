/*
 * ECE 153B
 *
 * Name(s): Justin Wu, Nikhil Vyas
 * Section: Tuesday 7PM
 * Project
 */
 
#include "DMA.h"
#include "SysTimer.h"

void DMA_Init_UARTx(DMA_Channel_TypeDef * tx, USART_TypeDef * uart) {
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; // enable DMA clock
	for (int i = 0; i < 1600; i++); // delay 20 us
	
	// Disable channel
	tx->CCR &= ~DMA_CCR_EN;
	// Disable memory-to-memory mode
	tx->CCR &= ~DMA_CCR_MEM2MEM;
	// Channel priority to high
	tx->CCR &= ~DMA_CCR_PL;
	tx->CCR |= DMA_CCR_PL_1; // (set to 10 for high)
	// Set peripheral size to 8 bit
	tx->CCR &= ~DMA_CCR_PSIZE;
	// Set memory size to 8 bit
	tx->CCR &= ~DMA_CCR_MSIZE;
	// Disable peripheral increment mode
	tx->CCR &= ~DMA_CCR_PINC;
	// Enable memory increment mode
	tx->CCR |= DMA_CCR_MINC;
	// Disable circular mode
	tx->CCR &= ~DMA_CCR_CIRC;
	// Set data transfer direction to Memory-to-Peripheral
	tx->CCR |= DMA_CCR_DIR;
	
	// Enable Transfer Complete interrupt
	tx->CCR |= DMA_CCR_TCIE;
	
	if (tx == DMA1_Channel4) {
		DMA1_CSELR->CSELR |= 0b0010 << 12;
		NVIC_SetPriority(DMA1_Channel4_IRQn, 1);
		NVIC_EnableIRQ(DMA1_Channel4_IRQn);
		NVIC_ClearPendingIRQ(DMA1_Channel4_IRQn);
	}
	else if (tx == DMA1_Channel7) {
		DMA1_CSELR->CSELR |= 0b0010 << 24;
		NVIC_SetPriority(DMA1_Channel7_IRQn, 1);
		NVIC_EnableIRQ(DMA1_Channel7_IRQn);
		NVIC_ClearPendingIRQ(DMA1_Channel7_IRQn);
	}
}
