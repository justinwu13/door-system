/*
 * ECE 153B
 *
 * Name(s): Justin Wu, Nikhil Vyas
 * Section: Tuesday 7PM
 * Project
 */


#include "UART.h"
#include "DMA.h"
#include "LED.h"
#include <stdio.h>
#include <string.h>

// DMA1 CH4: USART1_TX
// DMA1 CH5: USART1_RX
// DMA1 CH6: USART2_RX
// DMA1 CH7: USART2_TX

static volatile DMA_Channel_TypeDef * tx; 
static volatile char inputs[IO_SIZE]; // receive buffer
static volatile uint8_t receive_size = 0;
static volatile uint8_t data_t_0[IO_SIZE]; // transmit buffer 1
static volatile uint8_t data_t_1[IO_SIZE]; // transmit buffer 2
static volatile uint8_t input_size = 0;
static volatile uint8_t pending_size = 0;
static volatile uint8_t * active = data_t_0;
static volatile uint8_t * pending = data_t_1;

volatile uint8_t dmaDone = 0;

#define SEL_0 1
#define BUF_0_EMPTY 2
#define BUF_1_EMPTY 4
#define BUF_0_PENDING 8
#define BUF_1_PENDING 16

void transfer_data(char ch);
void on_complete_transfer(void);

void UART1_Init(void) {
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	RCC->CCIPR |= RCC_CCIPR_USART1SEL_0; // 01: SYSCLK
}

void UART2_Init(void) {
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
	RCC->CCIPR |= RCC_CCIPR_USART2SEL_0; // 01: SYSCLK
}

void UART1_GPIO_Init(void) {
	// setup PA9 and PA10
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	// set mode to alternate and set both pins to AF7 for UART functionality
	GPIOA->MODER &= ~GPIO_MODER_MODE9;
	GPIOA->MODER |= GPIO_MODER_MODE9_1;
	GPIOA->MODER &= ~GPIO_MODER_MODE10;
	GPIOA->MODER |= GPIO_MODER_MODE10_1;
	GPIOA->AFR[1] |= GPIO_AFRH_AFSEL9; // set to 1111
	GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL9_3; // reset 1000 to get to 0111 (AF7)
	GPIOA->AFR[1] |= GPIO_AFRH_AFSEL10;
	GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL10_3;
	
	// out speed to high
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED9;
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED10;
	
	// out type to push/pull
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT9;
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT10;
	
	// pull-up
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD9_0;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD10_0;
}

void UART2_GPIO_Init(void) {
	// setup PA2 and PA3
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	
	// set mode to alternate and set both pins to AF7 for UART functionality
	GPIOA->MODER &= ~GPIO_MODER_MODE2;
	GPIOA->MODER |= GPIO_MODER_MODE2_1;
	GPIOA->MODER &= ~GPIO_MODER_MODE3;
	GPIOA->MODER |= GPIO_MODER_MODE3_1;
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2; // set to 1111
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL2_3; // reset 1000 to get to 0111 (AF7)
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL3_3;
	
	// out speed to high
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED2;
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED3;
	
	// out type to push/pull
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT2;
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT3;
	
	// pull-up
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD2_0;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD3_0;
}

void USART_Init(USART_TypeDef * USARTx) {
	// Disable USART before configuring settings
	USARTx->CR1 &= ~USART_CR1_UE;
	
	// specific handling for USART1/2
	if (USARTx == USART1) {
		UART1_Init();
		UART1_GPIO_Init();
		tx = DMA1_Channel4; // USART1_TX

		// enable USART1 interrupt
		NVIC_SetPriority(USART1_IRQn, 2);
		NVIC_EnableIRQ(USART1_IRQn);
	}
	else if (USARTx == USART2) {
		UART2_Init();
		UART2_GPIO_Init();
		tx = DMA1_Channel7; // USART2_TX

		// enable USART2 interrupt
		NVIC_SetPriority(USART2_IRQn, 2);
		NVIC_EnableIRQ(USART2_IRQn);
	}
	
	DMA_Init_UARTx(tx, USARTx); // init DMA
	
	// Set data source to active
	tx->CMAR = (uint32_t) active;
	// Set destination to USART TDR
	tx->CPAR = (uint32_t) &USARTx->TDR;
	
	// Set Communication Parameters
	USARTx->CR1 &= ~(USART_CR1_M);     // 00 -> 8 Data Bits
	USARTx->CR1 &= ~(USART_CR1_OVER8); // 0 -> Oversampling by 16
	USARTx->CR2 &= ~(USART_CR2_STOP);  // 00 -> 1 Stop Bit
	
	// Set Baud Rate
	// f_CLK = 80 MHz, Baud Rate = 9600 = 80 MHz / DIV -> DIV = 8333 = 0x208D
	USARTx->BRR = 0x208D;
	
	// Enable Transmitter/Receiver
	USARTx->CR1 |= USART_CR1_TE | USART_CR1_RE;
	
	// Enable transmit DMA
	USARTx->CR3 |= USART_CR3_DMAT;
	
	// Prevents overrun from breaking UART
	USARTx->CR3 |= USART_CR3_OVRDIS;

	// enable receiver not empty interrupt
	USARTx->CR1 |= USART_CR1_RXNEIE;

	// Enable USART
	USARTx->CR1 |= USART_CR1_UE;
}

/**
 * This function accepts a string that should be sent through UART
*/
void UART_print(char* data) {
	//Transfer char array to buffer
	
	//Check DMA status. If DMA is ready, send data
	//If DMA is not ready, put the data aside
	if ((tx->CCR & DMA_CCR_EN) != DMA_CCR_EN) { // DMA not enabled, DMA ready
		input_size = strlen(data);
		sprintf(active, data);
		tx->CNDTR = input_size;
		dmaDone = 0;
		tx->CCR |= DMA_CCR_EN;
		while (dmaDone == 0) {} // wait until DMA done
		
		// clear active
		memset(active, 0, input_size);
		input_size = 0;
	}
	else { // move data into pending
		sprintf(pending + pending_size, data);
		pending_size += strlen(data);
	}
}

/**
 * This function should be invoked when a character is accepted through UART
*/
void transfer_data(char ch) {
	// Append character to input buffer.
	// If the character is end-of-line, invoke UART_onInput
	inputs[receive_size] = ch;
	receive_size++;
	if (ch == '\n') {
		// make copies of original data so we can clear them before entering wait state, may not be needed
		char copy[IO_SIZE];
		uint32_t rs_copy = receive_size;
		sprintf(copy, inputs);
		
		// clear inputs
		memset(inputs, 0, receive_size);
		receive_size = 0;
		
		UART_onInput(copy, rs_copy);
	}
}

/**
 * This function should be invoked when DMA transaction is completed
*/
void on_complete_transfer(void) {
	//TODO
	// If there are pending data to send, switch active and pending buffer, and send data
	if (pending_size != 0) {
		uint8_t* temp = active;
		active = pending;
		pending = temp;
		tx->CNDTR = pending_size;
		dmaDone = 0;
		tx->CCR |= DMA_CCR_EN;
		while (dmaDone == 0) {} // wait until DMA done
		tx->CCR &= ~DMA_CCR_EN;
			
		// clear pending
		memset(pending, 0, pending_size);
		pending_size = 0;
	}
}

/*
*	Check RXNE for new UART input while door is waiting to close/open
* USART2 for USB, USART1 for BT
*/
void UART_checkForInput(void) {
	if ((USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) { // if receive buffer is not empty. 
		transfer_data(USART2->RDR);
	}
}

void USART1_IRQHandler(void){
	// clear pending
	NVIC_ClearPendingIRQ(USART1_IRQn);
	tx = DMA1_Channel4;
	
	// When receive a character, invoke transfer_data
	// When complete sending data, invoke on_complete_transfer
	if ((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) { // receiving
		transfer_data(USART1->RDR);
	}
	// unused code, TCIE is off
	else if ((USART1->ISR & USART_ISR_TC) == USART_ISR_TC) { // transfer complete
		USART1->ICR |= USART_ICR_TCCF; // clear TC flag
		on_complete_transfer();
	}
}

void USART2_IRQHandler(void){
	tx = DMA1_Channel7;
	// clear pending 
	NVIC_ClearPendingIRQ(USART2_IRQn);
	
	// When receive a character, invoke transfer_data
	// When complete sending data, invoke on_complete_transfer
	if ((USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) { // receiving
		transfer_data(USART2->RDR);
	}
	// unused code, TCIE is off
	else if ((USART2->ISR & USART_ISR_TC) == USART_ISR_TC) { // transfer complete
		USART2->ICR |= USART_ICR_TCCF; // clear TC flag
		on_complete_transfer();
	}
}

void DMA1_Channel4_IRQHandler(void){ 
	// Clear NVIC interrupt flag
	NVIC_ClearPendingIRQ(DMA1_Channel4_IRQn);
	
	// Check if transfer is complete
	if ((DMA1->ISR & DMA_ISR_TCIF4) == DMA_ISR_TCIF4) {
		DMA1->IFCR |= DMA_IFCR_CTCIF4;
		dmaDone = 1;
		tx->CCR &= ~DMA_CCR_EN;
		on_complete_transfer();
	}
	
	// clear global DMA interrupt flag
	DMA1->IFCR |= DMA_IFCR_CGIF4;
}

void DMA1_Channel7_IRQHandler(void){ 
	// Clear NVIC interrupt flag
	NVIC_ClearPendingIRQ(DMA1_Channel7_IRQn);
	
	// Check if transfer is complete
	if ((DMA1->ISR & DMA_ISR_TCIF7) == DMA_ISR_TCIF7) {
		DMA1->IFCR |= DMA_IFCR_CTCIF7;
		dmaDone = 1;
		tx->CCR &= ~DMA_CCR_EN;
		on_complete_transfer();
	}
	
	// clear global DMA interrupt flag
	DMA1->IFCR |= DMA_IFCR_CGIF7;
}
