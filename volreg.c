#define F_CPU 14318000UL

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "ShiftOut_lib.h"
#include "EEPROM_lib.h"


//Encoder
#define encA PD3
#define encB PD4
#define encC PD2
//
#define TIME_TO_WRITE_EEPROM 1000   //время ожидания перед записью в еепром

//volatile uint8_t nextChar = 0;
volatile uint16_t   millis = 0,
                    sec = 0;
         
         
volatile uint16_t   currentRaw = 0,
                    temperatureRaw = 0,
                    temperatureChipRaw = 0;
int temperature = 0, 
    temperatureChip = 0, 
    current = 0;

volatile int8_t volume = 0;
volatile uint8_t volumeChanged = 0;
volatile uint32_t timeVolumeChanged = 0;

volatile uint8_t string[4] = {1, 10, 12, 13};

uint8_t chars[16] = {
  0x3F,  //0        0
  0x06,  //1        1
  0x5B,  //2        2
  0x4F,  //3        3
  0x66,  //4        4
  0x6D,  //5        5
  0x7D,  //6        6
  0x07,  //7        7
  0x7F,  //8        8
  0x6F,  //9        9
  0x00,  //space    10
  0x77,  //A        11
  0x5E,  //d        12
  0x7C,  //b        13
  0x63,  //deg      14
  0x39   //C        15
};


void setup(void) {
  InitShift();
  
  //DDRC |= (1 << PC1);
  DDRD &= ~((1 << PD2) | (1 << PD3) | (1 << PD4));
  PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD4);
  

  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (0 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0); // опорное напряжение - VCC, левое ориентирование данных, выбран вход ADC6
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // АЦП вкл, запуск, прерывание, частота CLK/128
  ADCSRB = 0x00; // режим автоизмерения: постоянно запущено  
  //DIDRO |= (1 << 7) | (1 <<6);

  TCCR0A = (1 << WGM01);  
  TCCR0B = (1 << CS01) | (1 << CS00);
  TIMSK0 = (1 << OCIE0A);
  OCR0A = 0xDF;
  
  TCCR1A = (1 << WGM21);  
  TCCR1B = (0 << CS12) | (0 << CS11) | (0 << CS10) | (1 << WGM12);
  TIMSK1 = (1 << OCIE1A);
  OCR1AH = 0xF0;
  OCR1AL = 0x01;
  
  
  EICRA = (1 << ISC11) | (0 << ISC10) | (1 << ISC01) | (1 << ISC00);
  EIMSK = (1 << INT1) | (1 << INT0);
  PCICR = 0x00;
  
  sei(); 
  
  
  EEPROM_write(0,50);
  volume = EEPROM_read(0);
}

void PrepareString(uint16_t _data, uint8_t _dataType){     //dataType  0 - 12.3`   1 - 12db    2 - 1.23A
    
    switch (_dataType){                              //data - целое число, для dataType=0 data/10, для 2 data/100
        case 0 :    //_data - трехзначное
            (_data/100 == 0) ? (string[0] = chars[10]) : (string[0] = chars[_data/100]);
            _data %= 100;
            string[1] = chars[_data/10] | (1 << 7);  // (1 << 7) ставит точку
            _data %= 10;
            string[2] = chars[_data];
            string[3] = chars[14];          //градус
            break;
        case 1 :    //_data - двузначное
            (_data/10 == 0) ? (string[0] = chars[10]) : (string[0] = chars[_data/10]);
            _data %= 10;
            string[1] = chars[_data];  
            _data %= 10;
            string[2] = chars[12];
            string[3] = chars[13];          //db
            break;
        case 2 :    //_data - трехзначное
            string[0] = chars[_data/100]  | (1 << 7);    // (1 << 7) ставит точку
            _data %= 100;
            string[1] = chars[_data/10];  
            _data %= 10;
            string[2] = chars[_data];
            string[3] = chars[11];          //А
            break;
    }
}    

int getTemperature(){   //преобразование темперауты с LM34 в градусы цельсия
                        //((значение с ацп(отсчетов) * 0,004483(в/отсчет) *1000(мв) /10(мв/F)) - 32) * 5/9
                        //и еще это *10 для отображения на дисплее
                        //еще добавил небольшой фильтр т.к 1 отсчет ацп = 0,3градС с фильтром не так скачет.
                        //да здравствуют градусы цельия!
    return (((temperatureRaw*0.4883 - 32)*5.5556) + 7 * temperature)/8;
}

int getCurrent(){
    if (currentRaw < 210) return 0;
    
    return ((currentRaw - 210)*0.495 + current*7) / 8;
}

int getTemperatureChip(){
    
    return (((temperatureChipRaw-314.31) / 1.22) + temperatureChip*7)/8;
}

void SelectString(){
    if (sec%9 < 3) PrepareString(temperature, 0);
    else if (sec%9 < 6) PrepareString(volume, 1);
    else PrepareString (current, 2);
}

ISR(ADC_vect){
    static uint8_t adcSource = 0x06;
    switch (adcSource) {
        case 6 :        //чтение тока
            currentRaw = ADCL ;
            currentRaw |= ADCH << 8;       //читаем данные из регистра
        break;
        case 7 :        //чтение температуры
            temperatureRaw = ADCL ;
            temperatureRaw |= ADCH << 8;   //читаем данные из регистра
        break;
        case 8 :        //чтение температуры
            temperatureChipRaw = ADCL ;
            temperatureChipRaw |= ADCH << 8;   //читаем данные из регистра
        break;
    }
    adcSource >= 8 ? adcSource = 6 : adcSource++;   //выбираем следующий канал
    adcSource == 8 ? (ADMUX |= (1 << REFS1)) : (ADMUX &= ~(1 << REFS1)); //опорное напряжение при измерении температуры чипа внутр. 1.1в, иначе 5в
    ADMUX &= 0xF0;
    ADMUX |= adcSource;                             
    ADCSRA |= (1 << ADSC);                          //запускаем следующее преобразование
    
}

ISR(TIMER0_COMPA_vect) {            //выводим символы по очереди 1 2 3 4  из массива string[]    
    static uint8_t nextChar = 0;    //static инициализируется только 1 раз, потом просто используется при вызову функции.
    nextChar >= 3 ? nextChar = 0 : nextChar++;
    uint16_t data = string[nextChar] << 8;    
    data |= (1 << nextChar); 
    ShiftOutDisplay(data);

    if (millis >= 1000){            //считаем времечко
        millis = 0;
        sec++;}
    else{
        millis++;}
    
}

ISR(TIMER1_COMPA_vect) {                
    TCCR1B &= ~((1 << CS12) | (0 << CS11) | (1 << CS10));
    EIMSK |= (1 << INT1);
    EIFR = (1 << INTF1);
}

ISR(INT0_vect){    
    (volume == 0) ? (volume = EEPROM_read(0)) : (volume = 0);   
    
    _delay_ms(5);
}

ISR(INT1_vect){
    EIMSK &= ~(1 << INT1);

    if (PIND & (1 << PD4)) volume--; else volume++;
    if (volume < 0)  volume = 0;
    if (volume > 96)  volume = 96;    
    volumeChanged = 1;
    timeVolumeChanged = sec*1000 + millis;
    _delay_ms(10);
    
    TCCR1B |= ((1 << CS12) | (0 << CS11) | (1 << CS10));


}


void main(void) {
    setup();        

    while (1) {        
                
//        temperature = getTemperature();
//        temperatureChip = getTemperatureChip();
//        current = getCurrent();
        
 /*       if (volumeChanged && ((sec*1000 + millis) - timeVolumeChanged) > TIME_TO_WRITE_EEPROM) {
            EEPROM_write(0, volume);
            volumeChanged = 0;
        }
*/        
        PrepareString(volume, 1);
        //SelectString();
        
        ShiftOutRelay(volume & 0xFF);
        
        _delay_ms(50);
/*        string[0] = chars[(temperature/1000)%10];
        string[1] = chars[(temperature/100)%10] | (1 << 7);
        string[2] = chars[(temperature/10)%10];
        string[3] = chars[temperature%10];
        
                string[0] = chars[(temperatureChip/1000)%10];
        string[1] = chars[(temperatureChip/100)%10] | (1 << 7);
        string[2] = chars[(temperatureChip/10)%10];
        string[3] = chars[temperatureChip%10];
        
                string[0] = chars[(current/1000)%10];
        string[1] = chars[(current/100)%10] | (1 << 7);
        string[2] = chars[(current/10)%10];
        string[3] = chars[current%10];*/
        
    }
}



