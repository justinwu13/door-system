#include "SPI.h"
#include "SysTimer.h"
#include "accelerometer.h"

void accWrite(uint8_t addr, uint8_t val){
	uint16_t transfer = ((addr << 8) | val);
	SPI_Transfer_Data(transfer);
}

uint8_t accRead(uint8_t addr){
	// access SPI_Transfer_Data
	uint16_t read_data = SPI_Transfer_Data((addr | 0x80) << 8); // R/W bit high for read operation
	uint8_t data = read_data & 0xFF;
	return data; // TODO
}

void initAcc(void){
	accWrite(0x2C, 0x0A); // write to BW_RATE register
	// set full range mode 
	accWrite(0x31, 0x08); // write to DATA_FORMAT register
	// enable measurement
	accWrite(0x2D, 0x08); // write to POWER_CTL register
}

void readValues(double* x, double* y, double* z){
	uint8_t x_0 = accRead(0x32);
	uint8_t x_1 = accRead(0x33);
	uint8_t y_0 = accRead(0x34);
	uint8_t y_1 = accRead(0x35);
	uint8_t z_0 = accRead(0x36);
	uint8_t z_1 = accRead(0x37);
	
	double x_data, y_data, z_data;
	if ((x_1 & 0x80) == 0x80) { // MSB is 1, x is negative
		x_data = -(0xFFFF ^ (x_1 << 8 | x_0) + 1); // two's complement
	}
	else {
		x_data = (x_1 << 8) | x_0;
	}
	
	if ((y_1 & 0x80) == 0x80) { // MSB is 1, y is negative
		y_data = -(0xFFFF ^ (y_1 << 8 | y_0) + 1); // two's complement
	}
	else {
		y_data = (y_1 << 8) | y_0;
	}
	
	if ((z_1 & 0x80) == 0x80) { // MSB is 1, z is negative
		z_data = -(0xFFFF ^ (z_1 << 8 | z_0) + 1); // two's complement
	}
	else {
		z_data = (z_1 << 8) | z_0;
	}
	
	// find scaler from data sheet
	double scaler = 3.9/1000; // 3.9 mg/LSB, or 3.9 g/1000 LSB
	// read values into x,y,z using accRead
	
	
	*x = x_data * scaler;
	*y = y_data * scaler;
	*z = z_data * scaler;

	/**
	*x = x_1;
	*y = x_0;
	*z = x_data;
	*/
}