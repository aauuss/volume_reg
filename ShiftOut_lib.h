#ifndef F_CPU
    #define F_CPU 14318000UL
#endif

#include <avr/io.h>
#include <util/delay.h>


//Relay
#define DS_R PC3
#define ST_R PC4
#define SH_R PC5
//Display
#define DS_D PD5
#define ST_D PD6
#define SH_D PD7


void InitShift();
void ShiftOutRelay(uint8_t _data);
void ShiftOutDisplay(uint16_t _data);

