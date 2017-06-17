#include "picduino.h"						//we use pic-port of the arduino libraries

//global defines
#define LED					PB5				//LED tied to pin 13/PB13
#define LED_DLY				100				//waste time, in ms


void setup(void) {
	//user setup code
	pinMode(LED, OUTPUT);					//LED as output
}

void loop(void) {
	//loops around here
	uint32_t time0, time1;

	digitalWrite(LED, !digitalRead(LED));	//flip the pin

	time0=ticks();
	//delay(LED_DLY);							//wasite some time
	while (ticks() - time0 < F_CPU * LED_DLY / 1000) continue;
	time1=ticks() - time0;
	if (time1) NOP();						//break point
}
