/*
 * pinout.h
 *
 * Created: 18-Jun-19 4:25:52 PM
 *  Author: Radek
 */ 

#ifndef PINOUT_H_
#define PINOUT_H_

//SPI
#define MISO			PB,01 //SERCOM-5 PAD[3]
#define CLK				PB,03 //SERCOM-5 PAD[1]
#define MOSI			PB,02 //SERCOM-5 PAD[0]
#define CS				PB,00 //SERCOM-5 PAD[2]

#define EX_1			PA,04 //SERCOM-0 PAD[0]
#define EX_2			PA,05 //SERCOM-0 PAD[1]
#define EX_3			PA,06 //SERCOM-0 PAD[2]
#define EX_4			PA,07 //SERCOM-0 PAD[3]
//LEDs
#define LED1			PA,00
#define LED2			PA,01
#define LED4			PB,23
#define LED5			PB,22

//Buttons
#define BTN1			PB,11
#define BTN2			PB,10

//Phase Control
#define P1_HS_LOGIC		PA,22
#define P2_HS_LOGIC		PA,21
#define P3_HS_LOGIC		PA,20

#define P1_LS_LOGIC		PA,18
#define P2_LS_LOGIC		PA,17
#define P3_LS_LOGIC		PA,16

//Phase Current Sense
#define I_PHASE_1		PA,10
#define I_PHASE_2		PA,9
#define I_PHASE_3		PA,8

//USB
#define USBP			PA,25
#define USBM			PA,24


#endif /* PINOUT_H_ */