#ifndef PIN_MACROS_H_
#define PIN_MACROS_H_


#define PA	0
#define PB	1
#define PC	2
#define PD	3

#define _SET(port,bit)	PORT->Group[port].OUTSET.reg=1<<(bit);
#define SET(x)		_SET(x)
#define _RESET(port,bit) PORT->Group[port].OUTCLR.reg=1<<(bit);
#define RESET(x)	_RESET(x)
#define _FLIP(port,bit)	PORT->Group[port].OUTTGL.reg=1<<(bit);
#define FLIP(x)		_FLIP(x)
#define _DIR_OUT(port,bit) PORT->Group[port].DIRSET.reg=1<<(bit);
#define DIR_OUT(x)	_DIR_OUT(x)
#define _DIR_IN(port,pin)	PORT->Group[port].DIRCLR.reg=1<<(pin);\
PORT->Group[port].PINCFG[pin].bit.INEN=1;
#define DIR_IN(x)	_DIR_IN(x)
#define _VALUE(port,bit)   (((PORT->Group[port].IN.reg)&(1<<(bit)))>>bit)
#define VALUE(x)	_VALUE(x)

#define _PORT_MUX(port,pin,value)	PORT->Group[port].PMUX[pin>>1].reg=(PORT->Group[port].PMUX[pin>>1].reg & ~(0xf<<(4*(pin&0x1))) )\
|value<<(4*(pin&0x1));\
PORT->Group[port].PINCFG[pin].bit.PMUXEN=1;
#define PORT_MUX(x,value) _PORT_MUX(x,value)

#define _PORT_ALT_FCN_ENABLE(port,pin)		PORT->Group[port].PINCFG[pin].bit.PMUXEN=1;
#define PORT_ALT_FCN_ENABLE(x)	_PORT_ALT_FCN_ENABLE(x)

#define _PORT_ALT_FCN_DISABLE(port,pin)		PORT->Group[port].PINCFG[pin].bit.PMUXEN=0;
#define PORT_ALT_FCN_DISABLE(x)	_PORT_ALT_FCN_DISABLE(x)



#define _PULLUP_ENABLE(port,pin)	PORT->Group[port].PINCFG[pin].bit.PULLEN=1;
#define PULLUP_ENABLE(x) _PULLUP_ENABLE(x)


#endif /* PIN_MACROS_H_ */
