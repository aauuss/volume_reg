#include "ShiftOut_lib.h"

void InitShift(){
    DDRC |= (1 << DS_R) | (1 << ST_R) | (1 << SH_R);
    PORTC &= ~((1 << DS_R) | (1 << ST_R) | (1 << SH_R));
    
    DDRD |= (1 << DS_D) | (1 << ST_D) | (1 << SH_D);
    PORTD &= ~((1 << DS_D) | (1 << ST_D) | (1 << SH_D));
}

void ShiftOutRelay(uint8_t _data) {
    _data = (_data << 2);
    PORTC &= ~(1 << DS_R);
    PORTC &= ~(1 << ST_R);
    PORTC &= ~(1 << SH_R);
    _delay_us(5);
    for (uint8_t i = 0; i < 8; i++) { 
        if ((1 << i) & (_data)) {
            PORTC |= (1 << DS_R);
        } else {
            PORTC &= ~(1 << DS_R);
        };
        _delay_us(5);        
        PORTC |= (1 << SH_R);
        _delay_us(5);
        PORTC &= ~(1 << DS_R);
        _delay_us(5);
        PORTC &= ~(1 << SH_R);
    }
    PORTC |= (1 << ST_R);
    _delay_us(5);
    PORTC &= ~(1 << ST_R);
    _delay_us(5);
    PORTC &= ~(1 << DS_R);
    PORTC &= ~(1 << ST_R);
    PORTC &= ~(1 << SH_R);
}


void ShiftOutDisplay(uint16_t _data) {
    PORTD &= ~(1 << DS_D);
    PORTD &= ~(1 << ST_D);
    PORTD &= ~(1 << SH_D);
    _delay_us(5);
    for (uint8_t i = 0; i < 16; i++) { 
        if ((1 << i) & (_data)) {
            PORTD |= (1 << DS_D);
        } else {
            PORTD &= ~(1 << DS_D);
        };
        _delay_us(5);        
        PORTD |= (1 << SH_D);
        _delay_us(5);
        PORTD &= ~(1 << DS_D);
        _delay_us(5);
        PORTD &= ~(1 << SH_D);
    }
    PORTD |= (1 << ST_D);
    _delay_us(5);
    PORTD &= ~(1 << ST_D);
    _delay_us(5);
    PORTD &= ~(1 << DS_D);
    PORTD &= ~(1 << ST_D);
    PORTD &= ~(1 << SH_D);
}
