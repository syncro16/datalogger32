#include <stdarg.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "edcMain.h"
#include "Serial.h"
#include "TempSensor.h"

/* phew! finally the main program rather than HAL crap */

extern SPI_HandleTypeDef hspi2;

void edcMain() {
	int i=0;
	serial.enableDmaRx();
	serial.printf("Datalogger32 ready\r\n");
	tempSensor.reset();
	TempSensorData *temp;
	while (true) {
		serial.ansiClearScreen();
		for (int i=1;i<9;i++) {
			temp = tempSensor.readChannel(i);
			serial.printf("Channel %d temp: %s %.2f %.2f \r\n",i,temp->error?"ERR":"OK ",temp->temperature,temp->internalTemperature);;
		}
//		serial.printf("Channel %d temp: %f\r\n",1,tempSensor.readChannel(1));
	
		HAL_Delay(1000/30);			
	}

//		serial.printf("i:%d\r\n",i);
//		serial.printf("Main: %d\r\n",i);
/*
		i++;
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET); 
		unsigned int data;
		HAL_Delay(1);		

		HAL_SPI_Receive(&hspi2,(unsigned char*)&data+3,1,1000);
		HAL_SPI_Receive(&hspi2,(unsigned char*)&data+2,1,1000);		
		HAL_SPI_Receive(&hspi2,(unsigned char*)&data+1,1,1000);			
		HAL_SPI_Receive(&hspi2,(unsigned char*)&data,1,1000);		
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET); 

		unsigned char c=0;
		for (int b=31;b>=0;b--){
			serial.printf((data & (1<<b))?"X":"-");
			c++;
			if (c%8==0) serial.printf(" ");
		}

		// ignore bottom 4 bits - they're just thermocouple data
		unsigned int v = data;
		v >>= 4;

		// pull the bottom 11 bits off
		float internal = v & 0x7FF;
		// check sign bit!
		if (v & 0x800) {
			// Convert to negative value by extending sign and casting to signed type.
			int16_t tmp = 0xF800 | (v & 0x7FF);
			internal = tmp;
		}
		internal *= 0.0625; // LSB = 0.0625 degrees
				

		v = data;

		if (v & 0x7) {
		// uh oh, a serious problem!
		}
//		v= 0xffffffff;

		if (v & 0x80000000) {
			// Negative value, drop the lower 18 bits and explicitly extend sign bits.
			v = 0xFFFFC000 | ((v >> 18) & 0x00003FFFF);
		} else {
			// Positive value, just drop the lower 18 bits.
			v >>= 18;
		}
			
		float centigrade = (signed int)v;

		// LSB = 0.25 degrees C
		centigrade *= 0.25;
//		centigrade = -1;

		serial.printf("temp %d %f %f\r\n",v,centigrade,internal);
	
		HAL_Delay(1000/60);		
		if (serial.available()) {
			serial.printf("Got: %c",serial.read());
		}	
	}
*/
}
