/*
 * ECE 153B
 *
 * Name(s): Justin Wu, Nikhil Vyas
 * Section: Tuesday 7PM
 * Project
 */

#include "stm32l476xx.h"
#include "SysClock.h"
#include "SysTimer.h"
#include "LED.h"
#include "DMA.h"
#include "UART.h"
#include "motor.h"
#include "SPI.h"
#include "I2C.h"
#include "accelerometer.h"
#include <stdio.h>
#include <string.h>

// PINS:
// SPI: PA4 = CS, PB3 = SCL, PB4 = SDO, PB5 = SDA; 3.3V
// I2C: PB6 = SCL, PB7 = SDA. Pinout: NC, SDA, GND, SCLK, VDD; 5V
// Motor: PC5, PC8, PC6, PC9 (IN1-4)
// UART1: RX -> PA9, TX -> PA10
// UART2: PA2, PA3 (no connection required)
// LED: PA5 (no connection required)

#define C_DIR 1 // clockwise to close
#define CC_DIR -1 // counterclockwise to open
#define STOP 0

// adjust as needed
#define ACCEL_CLOSE_THRESHOLD 1.05
#define ACCEL_OPEN_THRESHOLD 1.05

#define T_LOW 21
#define T_HIGH 26

static char buffer[IO_SIZE];
uint8_t beenInterrupted = 0; // becomes 1 if UART input comes in while door is in process of opening/closing
int prevTemp = 0; // store previous temperature to know to update on change
int currThreshold = 1; // 1 for T_HIGH being active, -1 for T_LOW being active

int onDelay = 0; // if the door is currently on delay after manual input. 0 = not on delay, 1 = on delay
int delayCounter = 0; // counts 0.1 sec periods, when this reaches 30 the door will no longer be on delay

void getTempData(int* temp);
void getAccelData(double* x, double* y, double* z);
void open();
void close();

void UART_onInput(char* inputs, uint32_t size) {
	if (onDelay) {
		sprintf(buffer, "Please wait until door ready to input.\r\n");
		UART_print(buffer);
	}
	else if (!strcmp(inputs, "o\n")) { // o - open
		open();
		if (!beenInterrupted) { // if the open/close operation completed (wasn't interrupted)
			beenInterrupted = 1;
			
			// 3s delay before door becomes ready again
			delayCounter = 0;
			onDelay = 1; 
		}
	}
	else if (!strcmp(inputs, "c\n")) { // c - close
		close();
		if (!beenInterrupted) {
			beenInterrupted = 1;
			
			// 3s delay before door becomes ready again
			delayCounter = 0;
			onDelay = 1; 
		}
	}
	else if (!strcmp(inputs, "s\n")) { // s - stop
		setDire(0);
		sprintf(buffer, "Door stopped.\r\n");
		UART_print(buffer);
		beenInterrupted = 1;
	}
	else { // anything else is invalid
		sprintf(buffer, "Invalid input.\r\n");
		UART_print(buffer);
	}
}

void checkTemperature(int temp) { // check whether temperature threshold is met
	if (onDelay) { // don't check temps if on delay
		return;
	}
	if (currThreshold > 0 && temp >= T_HIGH) { // door closed, temp too high
		sprintf(buffer, "Temperature of %d high, ", temp);
		UART_print(buffer);
		currThreshold *= -1;
		open();
		beenInterrupted = 1;
	}
	else if (currThreshold < 0 && temp <= T_LOW) { // door open, temp too low
		sprintf(buffer, "Temperature of %d low, ", temp);
		UART_print(buffer);
		currThreshold *= -1;
		close();
		beenInterrupted = 1;
	}
}

void close() {
	sprintf(buffer, "Closing\r\n");
	UART_print(buffer);
	setDire(C_DIR); 
	beenInterrupted = 0;
	double x, y, z;
	int temp;
	getAccelData(&x, &y, &z);
	while (!beenInterrupted && y < ACCEL_CLOSE_THRESHOLD) { // let door close until UART input or accelerometer detects completion
		delay(200);
		getAccelData(&x, &y, &z);
		getTempData(&temp);
		checkTemperature(temp);
		UART_checkForInput(); // check for manual input as that overrides current garage action
		
		sprintf(buffer, "Acceleration: %.2f, %.2f, %.2f\r\n", x, y, z);
		UART_print(buffer);
	}
	setDire(STOP);
	if (!beenInterrupted) {
		sprintf(buffer, "Done closing\r\n");
		UART_print(buffer);
		currThreshold = 1;
	}
}

void open() {
	sprintf(buffer, "Opening\r\n");
	UART_print(buffer);
	setDire(CC_DIR);
	beenInterrupted = 0;
	double x, y, z;
	int temp;
	getAccelData(&x, &y, &z);
	while (!beenInterrupted && z < ACCEL_OPEN_THRESHOLD) { // let door open until UART input or accelerometer detects completion
		getAccelData(&x, &y, &z);
		getTempData(&temp);
		checkTemperature(temp);
		UART_checkForInput(); // check for manual input as that overrides current garage action
		
		sprintf(buffer, "Acceleration: %.2f, %.2f, %.2f\r\n", x, y, z);
		UART_print(buffer);
		delay(100);
	}
	delay(800); // delay to account for accelerometer stopping early
	setDire(STOP);
	if (!beenInterrupted) {
		sprintf(buffer, "Done opening\r\n");
		UART_print(buffer);
		currThreshold = -1;
	}
}

void getTempData(int* temp) {
	// Note the "<< 1" must be present because bit 0 is treated as a don't care in 7-bit addressing mode
	uint8_t SecondaryAddress = 0b1001000 << 1;
	uint8_t Data_Send = 0x00;
	uint8_t Data_Receive;
	
	//*temp = 24;
	
	// Get Temperature
	Data_Send = 0x00;
	I2C_SendData(I2C1, SecondaryAddress, &Data_Send, 1); 
	// First, send a command to the sensor for reading the temperature
	// Next, get the measurement
	I2C_ReceiveData(I2C1, SecondaryAddress, &Data_Receive, 1);
	
	/*
		// Print Temperature to Termite
	uint8_t sign_bit = (0x80 & Data_Receive) == 0x80;
	
	if (sign_bit == 1) { // if first bit is a 1, the temp is negative
		uint8_t no_sign_data = (0x7F ^ Data_Receive) + 1;
		sprintf(buffer, "-%d C\n", no_sign_data);
	}
	else {
		uint8_t no_sign_data = 0x7F & Data_Receive;
		sprintf(buffer, "%d C\n", no_sign_data);
	}	
	UART_print(buffer);
	*/
	*temp = 0x7F & Data_Receive;
	
}

void getAccelData(double* x, double* y, double* z) {
	readValues(x, y, z);
	/*
	sprintf(buffer, "Acceleration: %.2f, %.2f, %.2f\r\n", x, y, z);
	UART_print(buffer);
	*/
}

int main(void) {
	// Switch System Clock = 80 MHz
	System_Clock_Init(); 
	Motor_Init();
	SysTick_Init();
	LED_Init();	
	USART_Init(USART1); // change to USART1 for BT, USART2 for USB
	
	SPI1_GPIO_Init();
	SPI1_Init();
	initAcc();
	I2C_GPIO_Init();
	I2C_Initialization();
	
	sprintf(buffer, "Program Starts.\r\n");
	UART_print(buffer);
	int temp = 0;
	double x, y, z;
	
	close(); // make sure door starts closed
	while(1) {
		if (onDelay) { // if door has just been opened/closed manually
			if (delayCounter >= 30) { // 30 loops * 0.1s delay per loop = 3s
				onDelay = 0;
				delayCounter = 0;
				sprintf(buffer, "Door ready.\r\n");
				UART_print(buffer);
			}
			else {
				delayCounter++;
			}
		}
		getAccelData(&x, &y, &z);
		getTempData(&temp);
		if (prevTemp != temp) {
			sprintf(buffer, "Temperature = %d C\n", temp);
			UART_print(buffer);
			
		}
		prevTemp = temp;
		checkTemperature(temp);
		delay(100);
	}
}


