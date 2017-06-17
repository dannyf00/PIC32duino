//PICduino16 code base

#include "picduino_config.h"				//configuration words - for XC16
#include "picduino.h"						//PICduino-related definitions / prototypes

//empty handler
static void empty_handler(void) {
	//default tmr handler - do nothing here
}

//global variables

#define _T1IF		IFS0bits.T1IF
#define _T1IE		IEC0bits.T1IE


//global variables
//for time base off TIMER1 @ 1:1 prescaler
//volatile uint32_t timer1_millis = 0;
volatile uint32_t timer_ticks = 0;
//static uint16_t timer1_fract = 0;

//timer1 overflow isr
//void _ISR _T1Interrupt(void) {
void __ISR(_TIMER_1_VECTOR/*, ipl7AUTO*/) _T1Interrupt(void) {
	_T1IF=0;							//clear tmr1 interrupt flag
	timer_ticks+=0x10000ul;				//increment overflow count: 16-bit timer
}

//declare pins
//ALL PINS ARE MAPPED, WHETHER THEY EXIST OR NOT
//SO MAKE SURE THAT THE PINS YOU PICKED ACTUALLY EXIST FOR YOUR PACKAGE
//Pin  0.. 7 -> GPIOA
//Pin  8..15 -> GPIOB
//Pin 16..23 -> GPIOC
//Pin 24..31 -> GPIOD
//Pin 32..39 -> GPIOE
//Pin 40..47 -> GPIOF
//Pin 48..55 -> GPIOG
//Pin 56..63 -> GPIOH
//Pin 64..71 -> GPIOI
const PIN2GPIO GPIO_PinDef[]={
	{GPIOA, 1<<0},						//PICduino Pin  0 = RP0/PB0/CHIP PIN4
	{GPIOA, 1<<1},						//PICduino Pin  1 = RP1/PB1/CHIP PIN5
	{GPIOA, 1<<2},						//PICduino Pin  2 = RP2/PB2/CHIP PIN6
	{GPIOA, 1<<3},						//PICduino Pin  3 = RP3/PB3/CHIP PIN7
	{GPIOA, 1<<4},						//PICduino Pin  4 = RP4/PB4/CHIP PIN11
	{GPIOA, 1<<5},						//PICduino Pin  5 = RP5/PB5/CHIP PIN14
	{GPIOA, 1<<6},						//PICduino Pin  6 = RP6/PB6/CHIP PIN15
	{GPIOA, 1<<7},						//PICduino Pin  7 = RP7/PB7/CHIP PIN16
	{GPIOA, 1<<8},						//PICduino Pin  8 = RP8/PB8/CHIP PIN17
	{GPIOA, 1<<9},						//PICduino Pin  9 = RP9/PB9/CHIP PIN18
	{GPIOA, 1<<10},						//PICduino Pin 10 = RP10/PB10/CHIP PIN21
	{GPIOA, 1<<11},						//PICduino Pin 11 = RP11/PB11/CHIP PIN22
	{GPIOA, 1<<12},						//PICduino Pin 12 = RP12/PB12/CHIP PIN23
	{GPIOA, 1<<13},						//PICduino Pin 13 = RP13/PB13/CHIP PIN24
	{GPIOA, 1<<14},						//PICduino Pin 14 = RP14/PB14/CHIP PIN25
	{GPIOA, 1<<15},						//PICduino Pin 15 = RP15/PB15/CHIP PIN26

	{GPIOB, 1<<0},						//PICduino Pin 16 = RP0/PB0/CHIP PIN4
	{GPIOB, 1<<1},						//PICduino Pin 17 = RP1/PB1/CHIP PIN5
	{GPIOB, 1<<2},						//PICduino Pin 18 = RP2/PB2/CHIP PIN6
	{GPIOB, 1<<3},						//PICduino Pin 19 = RP3/PB3/CHIP PIN7
	{GPIOB, 1<<4},						//PICduino Pin 20 = RP4/PB4/CHIP PIN11
	{GPIOB, 1<<5},						//PICduino Pin 21 = RP5/PB5/CHIP PIN14
	{GPIOB, 1<<6},						//PICduino Pin 22 = RP6/PB6/CHIP PIN15
	{GPIOB, 1<<7},						//PICduino Pin 23 = RP7/PB7/CHIP PIN16
	{GPIOB, 1<<8},						//PICduino Pin 24 = RP8/PB8/CHIP PIN17
	{GPIOB, 1<<9},						//PICduino Pin 25 = RP9/PB9/CHIP PIN18
	{GPIOB, 1<<10},						//PICduino Pin 26 = RP10/PB10/CHIP PIN21
	{GPIOB, 1<<11},						//PICduino Pin 27 = RP11/PB11/CHIP PIN22
	{GPIOB, 1<<12},						//PICduino Pin 28 = RP12/PB12/CHIP PIN23
	{GPIOB, 1<<13},						//PICduino Pin 29 = RP13/PB13/CHIP PIN24
	{GPIOB, 1<<14},						//PICduino Pin 30 = RP14/PB14/CHIP PIN25
	{GPIOB, 1<<15},						//PICduino Pin 31 = RP15/PB15/CHIP PIN26

#if defined(GPIOC)
	{GPIOC, 1<<0},						//PICduino Pin 32 = RP0/PB0/CHIP PIN4
	{GPIOC, 1<<1},						//PICduino Pin 33 = RP1/PB1/CHIP PIN5
	{GPIOC, 1<<2},						//PICduino Pin 34 = RP2/PB2/CHIP PIN6
	{GPIOC, 1<<3},						//PICduino Pin 35 = RP3/PB3/CHIP PIN7
	{GPIOC, 1<<4},						//PICduino Pin 36 = RP4/PB4/CHIP PIN11
	{GPIOC, 1<<5},						//PICduino Pin 37 = RP5/PB5/CHIP PIN14
	{GPIOC, 1<<6},						//PICduino Pin 38 = RP6/PB6/CHIP PIN15
	{GPIOC, 1<<7},						//PICduino Pin 39 = RP7/PB7/CHIP PIN16
	{GPIOC, 1<<8},						//PICduino Pin 40 = RP8/PB8/CHIP PIN17
	{GPIOC, 1<<9},						//PICduino Pin 41 = RP9/PB9/CHIP PIN18
	{GPIOC, 1<<10},						//PICduino Pin 42 = RP10/PB10/CHIP PIN21
	{GPIOC, 1<<11},						//PICduino Pin 43 = RP11/PB11/CHIP PIN22
	{GPIOC, 1<<12},						//PICduino Pin 44 = RP12/PB12/CHIP PIN23
	{GPIOC, 1<<13},						//PICduino Pin 45 = RP13/PB13/CHIP PIN24
	{GPIOC, 1<<14},						//PICduino Pin 46 = RP14/PB14/CHIP PIN25
	{GPIOC, 1<<15},						//PICduino Pin 47 = RP15/PB15/CHIP PIN26
#endif
};

	
//Arduino Functions: GPIO
//set a pin mode to INPUT or OUTPUT
//no error checking on PIN
inline void pinMode(PIN_TypeDef pin, uint8_t mode) {
	if (mode==INPUT) GIO_IN(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
	else GIO_OUT(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
}

//set / clear a pin
inline void digitalWrite(PIN_TypeDef pin, uint8_t val) {
	if (val==LOW) GIO_CLR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
	else GIO_SET(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
}

//read a pin
inline int digitalRead(PIN_TypeDef pin) {
	return (GIO_GET(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask))?HIGH:LOW;
}
//end GPIO

//Arduino Functions: Time
//return timer ticks
uint32_t ticks(void) {
	uint32_t m;
	uint16_t f;
	
	do {
		m = timer_ticks;
		f = TMR1;
	} while (m != timer_ticks);
	//now m and f are consistent
	return (m | f);
}		

//return microseconds
uint32_t micros(void) {
	uint32_t m;					//stores overflow count
	uint16_t f;					//return the fractions / TMR1 value
	
	//use double reads
	do {
		m = timer_ticks;
		f = TMR1;
	} while (m != timer_ticks);
	//now m and f are consistent
	return (m | f) / clockCyclesPerMicrosecond() / 1;
}
	
//return milliseconds
//alternatively, = micros()/1000
uint32_t millis(void) {
	uint32_t m;
	uint16_t f;
	
	//use double reads
	do {
		m = timer_ticks;
		f = TMR1;
	} while (m != timer_ticks);
		
	return (m | f) / clockCyclesPerMillisecond();	//or shift 10 positions
}

//delay millisseconds
void delay(uint32_t ms) {
	uint32_t start_time = millis();

	while (millis() - start_time < ms) continue;
}

//delay micros seconds
void delayMicroseconds(uint32_t us) {
	uint32_t start_time = micros();
	
	while (micros() - start_time < us) continue;
}
//end Time


//Arduino Functions: Advanced IO
//shift in - from arduino code base / not optimized
uint8_t shiftIn(PIN_TypeDef dataPin, PIN_TypeDef clockPin, uint8_t bitOrder) {
	uint8_t value = 0;
	uint8_t i;

	for (i = 0; i < 8; ++i) {
		digitalWrite(clockPin, HIGH);
		if (bitOrder == LSBFIRST)
			value |= digitalRead(dataPin) << i;
		else
			value |= digitalRead(dataPin) << (7 - i);
		digitalWrite(clockPin, LOW);
	}
	return value;
}

//shift out - from arduino code base / not optimized
void shiftOut(PIN_TypeDef dataPin, PIN_TypeDef clockPin, uint8_t bitOrder, uint8_t val) {
	uint8_t i;

	for (i = 0; i < 8; i++)  {
		if (bitOrder == LSBFIRST)
			digitalWrite(dataPin, !!(val & (1 << i)));
		else	
			digitalWrite(dataPin, !!(val & (1 << (7 - i))));
			
		digitalWrite(clockPin, HIGH);
		digitalWrite(clockPin, LOW);		
	}
}

//wait for a pulse and return timing
uint32_t pulseIn(PIN_TypeDef pin, uint8_t state) {
	uint32_t tmp;
	state = (state == LOW)?LOW:HIGH;				
	while (digitalRead(pin) == state) continue;		//wait for the pin to opposite
	//now pin is _state
	tmp = micros();
	state = (state == LOW)?HIGH:LOW;				//calculate the state to end the wait
	while (digitalRead(pin) == state) continue;		//wait for the pin to go back to its original state
	tmp = micros() - tmp;							//calculate the pulse width
	return tmp;
}
//end Advanced IO

//reset the mcu
//start timer1 at 1:1 prescaler
void mcu_init(void) {
	//set PBDIV to 1:1
	//unlock sequency
	SYSKEY = 0x0; 									// ensure OSCCON is locked
	SYSKEY = 0xAA996655; 							// Write Key1 to SYSKEY
	SYSKEY = 0x556699AA; 							// Write Key2 to SYSKEY
	// OSCCON is now unlocked
	// make the desired change
	OSCCON = (OSCCON &~0x00180000ul) | (0x00 & 0x00180000ul);	//or to set through config bits
	//lock sequency
	SYSKEY = 0x00;									//write any value to lock
	
	//turn off all peripherals
	PMD1=PMD2=PMD3=PMD4=PMD5=PMD6=0xfffffffful; 

	//all pins digital
//#if defined(AD1PCFG)
	//AD1PCFG = 0xffff;						//1->all pins digital
	ANSELA = ANSELB = 0;					//0->all pins digital
#if defined(ANSELC)
	ANSELC = 0;
#endif

//#endif
	//or AD1PCFGH
//#if defined(AD1PCFGH)
	//AD1PCFGH = 0xffff;					//all pins digital
//#endif
	//or AD1PCFGL
//#if defined(AD1PCFGL)
	//AD1PCFGL = 0xffff;					//all pins digital
//#endif

	//initialize timer1 as time base
	//initialize timer_ticks
	timer_ticks=0;
	
	//configure timer1 as time base
	PMD4bits.T1MD = 0;						//enable power to tmr1
	T1CONbits.TON = 0;						//turn off rtc1
	PR1 = 0xffff;							//PR1=period-1;						//minimum rtc resolution is 1ms
	T1CONbits.TCS = 0;						//use internal clock = Fosc / 2
	T1CONbits.TCKPS=0x00;					//prescaler = 0b00 -> 1:1			//ps & TMR_PS_MASK;			//set prescaler to 1:1
	T1CONbits.TGATE = 0;					//rtc1 gate disabled
	//T1CONbits.T32 = 0;						//16-bit mode
	TMR1 = 0;								//reset the timer/counter
	_T1IF = 0;								//reset the flag
	_T1IE = 1;								//tmr1 interrupt on
	IPC1bits.T1IP = 2;
	IPC1bits.T1IS = 0;
	T1CONbits.TON = 1;						//turn on tmr1
	
	//disable interrupts
	//__builtin_disable_interrupts();			//disable interrupts
	//INTDisableInterrupts();
	//enable multivector mode
	//INTCONbits.MVEC = 1;					//1=enable multiple vector
	//__builtin_enable_interrupts();			//enable interrupts
	//INTEnableInterrupts();					
	//INTEnableSystemMultiVectoredInt();
	INTCONbits.MVEC = 1;					//1=enable multi-vectored interrupts, 0=disable

	//enable all interrupts
	interrupts();
}

//templated code for main()
int main(void) {
	mcu_init();							//reset the chip
	setup();							//user-set up the code
	while (1) {
		loop();							//user specified loop
	}
	return 0;
}
		