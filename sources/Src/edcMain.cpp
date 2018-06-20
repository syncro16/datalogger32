#include <stdarg.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "edcMain.h"
#include "Serial.h"
/* phew! finally the main program rather than HAL crap */

void edcMain() {
	int i=0;
	serial.enableDmaRx();
	serial.printf("Datalogger32 ready\r\n");
	while (true) {
//		serial.printf("i:%d\r\n",i);
//		serial.printf("Main: %d\r\n",i);
		i++;
		HAL_Delay(1000/60);		
		if (serial.available()) {
			serial.printf("Got: %c",serial.read());
		}	
	}
}
