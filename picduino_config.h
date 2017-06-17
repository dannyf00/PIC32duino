#ifndef __PICDUINO_CONFIG_H
#define	__PICDUINO_CONFIG_H

//#include <p32xxxx.h>							//p32xxxx.h included here
#include <xc.h>

//configuration settings
//PIC32MX110/120/130/150/170, PIC32MX210/220/230/250/270

#if   	defined(__32MX110F016B_H) || defined(__32MX110F016C_H) || defined(__32MX110F016D_H) || \
		defined(__32MX120F032B_H) || defined(__32MX120F032C_H) || defined(__32MX120F032D_H) || defined(__32MX120F064H_H) || \
		defined(__32MX130F064B_H) || defined(__32MX130F064C_H) || defined(__32MX130F064D_H) || defined(__32MX130F128H_H) || defined(__32MX130F128L_H) || \
		defined(__32MX150F128B_H) || defined(__32MX150F128C_H) || defined(__32MX150F128D_H) || defined(__32MX150F256H_H) || defined(__32MX150F256L_H) || \
		defined(__32MX170F256B_H) || defined(__32MX170F256D_H) || defined(__32MX170F256H_H) || defined(__32MX170F256L_H) || \
		defined(__32MX210F016B_H) || defined(__32MX210F016C_H) || defined(__32MX210F016D_H) || \
		defined(__32MX220F032B_H) || defined(__32MX220F032C_H) || defined(__32MX220F032D_H) || \
		defined(__32MX230F064B_H) || defined(__32MX230F064C_H) || defined(__32MX230F064D_H) || defined(__32MX230F128H_H) || defined(__32MX230F128L_H) || \
		defined(__32MX250F128B_H) || defined(__32MX250F128C_H) || defined(__32MX250F128D_H) || defined(__32MX250F256H_H) || defined(__32MX250F256L_H) || \
		defined(__32MX270F256B_H) || defined(__32MX270F256D_H) || defined(__32MX270F256H_H) || defined(__32MX270F256L_H)

#pragma config FNOSC = FRC						//oscillator selection bits = FRC/FRCPLL/PRI/PRIPLL/SOSC/LPRC/FRCDIV16/FRCDIV, 
#pragma config FSOSCEN = OFF					//secondary oscillator off
#pragma config IESO = OFF						//internal/external switch-over
#pragma config POSCMOD = HS						//primary oscilator configuration = OFF/HS/XT/EC
//PLL configuration.
//F_SYSCLK = F_OSC / FPLLIDIV * FPLLMUL / FPLLODIV.
#pragma config FPLLIDIV = DIV_2					//PLL input divider=DIV_1/2/3/4/5/6/10/12 (4Mhz < F_OSC / FPLLIDIV < 5Mhz)
#pragma config FPLLMUL = MUL_16					//PLL multiplier=MUL_15/16/17/18/19/20/21/24
#pragma config FPLLODIV = DIV_2					//PLL output divider=DIV_1/2/4/8/16/32/64/256
//end PLL configuration
//F_PBDIV = F_SYSCLK / FPBDIV
#pragma config FPBDIV = DIV_8					//peripheral bus clock divider = 8x/4x/2x/1x
#pragma config OSCIOFNC = OFF, FCKSM = CSECMD	//clock output disabled, clock switching disabled
#pragma config ICESEL = RESERVED				//use PGD1/PGC1, 
#pragma config PMDL1WAY = OFF, IOL1WAY = OFF	//peripheral configuration allows multiple configuration (OFF) or one configuration (ON), PPS allows multiple configuration (OFF)/one configuration(ON)
#pragma config FUSBIDIO = OFF, FVBUSONIO = OFF	//USBID pin controoled by port functions (OFF)/or USB module (ON), Vbuson pin controlled by port function (OFF)/USB module (ON)
#pragma config FWDTEN = OFF, WDTPS = PS32768	//watchdog timer disabled, watchdog timer prescaler = 32768
#pragma config WINDIS = OFF, FWDTWINSZ = WINSZ_75		//watchdog timer window disabled, watchdog timer window size =75%
#pragma config PWP = OFF, BWP = OFF, CP = OFF	//boot flash write-protection off, power-on protection off, code protection off
//#pragma config DEBUG = OFF						//OFF = debugger disabled. ON = debugger enabled -> *****needs to be unchecked to run code under pic32prog + pickit2*****
#pragma config JTAGEN = OFF						//jtag off

//set up usb divider
#if		defined(__32MX210F016B_H) || defined(__32MX210F016C_H) || defined(__32MX210F016D_H) || \
		defined(__32MX220F032B_H) || defined(__32MX220F032C_H) || defined(__32MX220F032D_H) || \
		defined(__32MX230F064B_H) || defined(__32MX230F064C_H) || defined(__32MX230F064D_H) || defined(__32MX230F128H_H) || defined(__32MX230F128L_H) || \
		defined(__32MX250F128B_H) || defined(__32MX250F128C_H) || defined(__32MX250F128D_H) || defined(__32MX250F256H_H) || defined(__32MX250F256L_H) || \
		defined(__32MX270F256B_H) || defined(__32MX270F256D_H) || defined(__32MX270F256H_H) || defined(__32MX270F256L_H)
#pragma config UPLLIDIV = DIV_1, UPLLEN = OFF	//USB PLL div=1, USB PLL enable=off/on
#endif

/*PIC32MX130F064B
Peripheral Module Disable Configuration:
 PMDL1WAY = OFF Allow multiple reconfigurations
PMDL1WAY = ON Allow only one reconfiguration

Peripheral Pin Select Configuration:
 IOL1WAY = OFF Allow multiple reconfigurations
IOL1WAY = ON Allow only one reconfiguration

USB USID Selection:
 FUSBIDIO = OFF Controlled by Port Function
FUSBIDIO = ON Controlled by the USB Module

USB VBUS ON Selection:
 FVBUSONIO = OFF Controlled by Port Function
FVBUSONIO = ON Controlled by USB Module

PLL Input Divider:
 FPLLIDIV = DIV_1 1x Divider
FPLLIDIV = DIV_2 2x Divider
FPLLIDIV = DIV_3 3x Divider
FPLLIDIV = DIV_4 4x Divider
FPLLIDIV = DIV_5 5x Divider
FPLLIDIV = DIV_6 6x Divider
FPLLIDIV = DIV_10 10x Divider
FPLLIDIV = DIV_12 12x Divider

PLL Multiplier:
 FPLLMUL = MUL_15 15x Multiplier
FPLLMUL = MUL_16 16x Multiplier
FPLLMUL = MUL_17 17x Multiplier
FPLLMUL = MUL_18 18x Multiplier
FPLLMUL = MUL_19 19x Multiplier
FPLLMUL = MUL_20 20x Multiplier
FPLLMUL = MUL_21 21x Multiplier
FPLLMUL = MUL_24 24x Multiplier

System PLL Output Clock Divider:
 FPLLODIV = DIV_1 PLL Divide by 1
FPLLODIV = DIV_2 PLL Divide by 2
FPLLODIV = DIV_4 PLL Divide by 4
FPLLODIV = DIV_8 PLL Divide by 8
FPLLODIV = DIV_16 PLL Divide by 16
FPLLODIV = DIV_32 PLL Divide by 32
FPLLODIV = DIV_64 PLL Divide by 64
FPLLODIV = DIV_256 PLL Divide by 256

Oscillator Selection Bits:
 FNOSC = FRC Fast RC Osc (FRC)
FNOSC = FRCPLL Fast RC Osc with PLL
FNOSC = PRI Primary Osc (XT,HS,EC)
FNOSC = PRIPLL Primary Osc w/PLL (XT+,HS+,EC+PLL)
FNOSC = SOSC Low Power Secondary Osc (SOSC)
FNOSC = LPRC Low Power RC Osc (LPRC)
FNOSC = FRCDIV16 Fast RC Osc w/Div-by-16 (FRC/16)
FNOSC = FRCDIV Fast RC Osc w/Div-by-N (FRCDIV)

Secondary Oscillator Enable:
 FSOSCEN = OFF Disabled
FSOSCEN = ON Enabled

Internal/External Switch Over:
 IESO = OFF Disabled
IESO = ON Enabled

Primary Oscillator Configuration:
 POSCMOD = EC External clock mode
POSCMOD = XT XT osc mode
POSCMOD = HS HS osc mode
POSCMOD = OFF Primary osc disabled

CLKO Output Signal Active on the OSCO Pin:
 OSCIOFNC = ON Enabled
OSCIOFNC = OFF Disabled

Peripheral Clock Divisor:
 FPBDIV = DIV_1 Pb_Clk is Sys_Clk/1
FPBDIV = DIV_2 Pb_Clk is Sys_Clk/2
FPBDIV = DIV_4 Pb_Clk is Sys_Clk/4
FPBDIV = DIV_8 Pb_Clk is Sys_Clk/8

Clock Switching and Monitor Selection:
 FCKSM = CSECME Clock Switch Enable, FSCM Enabled
FCKSM = CSECMD Clock Switch Enable, FSCM Disabled
FCKSM = CSDCMD Clock Switch Disable, FSCM Disabled

Watchdog Timer Postscaler:
 WDTPS = PS1 1:1
WDTPS = PS2 1:2
WDTPS = PS4 1:4
WDTPS = PS8 1:8
WDTPS = PS16 1:16
WDTPS = PS32 1:32
WDTPS = PS64 1:64
WDTPS = PS128 1:128
WDTPS = PS256 1:256
WDTPS = PS512 1:512
WDTPS = PS1024 1:1024
WDTPS = PS2048 1:2048
WDTPS = PS4096 1:4096
WDTPS = PS8192 1:8192
WDTPS = PS16384 1:16384
WDTPS = PS32768 1:32768
WDTPS = PS65536 1:65536
WDTPS = PS131072 1:131072
WDTPS = PS262144 1:262144
WDTPS = PS524288 1:524288
WDTPS = PS1048576 1:1048576

Watchdog Timer Window Enable:
 WINDIS = ON Watchdog Timer is in Window Mode
WINDIS = OFF Watchdog Timer is in Non-Window Mode

Watchdog Timer Enable:
 FWDTEN = OFF WDT Disabled (SWDTEN Bit Controls)
FWDTEN = ON WDT Enabled

Watchdog Timer Window Size:
 FWDTWINSZ = WINSZ_75 Window Size is 75%
FWDTWINSZ = WINSZ_50 Window Size is 50%
FWDTWINSZ = WINSZ_37 Window Size is 37.5%
FWDTWINSZ = WISZ_25 Window Size is 25%

Background Debugger Enable:
 DEBUG = ON Debugger is Enabled
DEBUG = OFF Debugger is Disabled

JTAG Enable:
 JTAGEN = OFF JTAG Disabled
JTAGEN = ON JTAG Port Enabled

ICE/ICD Comm Channel Select:
 ICESEL = RESERVED Reserved
ICESEL = ICS_PGx3 Communicate on PGEC3/PGED3
ICESEL = ICS_PGx2 Communicate on PGEC2/PGED2
ICESEL = ICS_PGx1 Communicate on PGEC1/PGED1

Program Flash Write Protect:
 PWP = PWP32K First 32K
PWP = PWP31K First 31K
PWP = PWP30K First 30K
PWP = PWP29K First 29K
PWP = PWP28K First 28K
PWP = PWP27K First 27K
PWP = PWP26K First 26K
PWP = PWP25K First 25K
PWP = PWP24K First 24K
PWP = PWP23K First 23K
PWP = PWP22K First 22K
PWP = PWP21K First 21K
PWP = PWP20K First 20K
PWP = PWP19K First 19K
PWP = PWP18K First 18K
PWP = PWP17K First 17K
PWP = PWP16K First 16K
PWP = PWP15K First 15K
PWP = PWP14K First 14K
PWP = PWP13K First 13K
PWP = PWP12K First 12K
PWP = PWP11K First 11K
PWP = PWP10K First 10K
PWP = PWP9K First 9K
PWP = PWP8K First 8K
PWP = PWP7K First 7K
PWP = PWP6K First 6K
PWP = PWP5K First 5K
PWP = PWP4K First 4K
PWP = PWP3K First 3K
PWP = PWP2K First 2K
PWP = PWP1K First 1K
PWP = OFF Disable

Boot Flash Write Protect bit:
 BWP = ON Protection Enabled
BWP = OFF Protection Disabled

Code Protect:
 CP = ON Protection Enabled
CP = OFF Protection Disabled
*/

#else                               //no chip identified
//no header file found
#warning no device header file specified in config.h
#endif

#endif	/* CONFIG_H */

