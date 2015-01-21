/*
  ldr.c - analog read methods
  Part of Horus Firmware

  Copyright (c) 2015 Irene Sanz (Mundo Reader S.L.)

  Horus is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Horus is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Horus.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ldr.h"

void ldr_init(void){
 ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));    //16Mhz/128 = 125Khz the ADC reference clock
 ADMUX |= (1<<REFS0);                //Voltage reference from Avcc (5v)
 ADCSRA |= (1<<ADEN);                //Turn on ADC
 ADCSRA |= (1<<ADSC);                //Do an initial conversion because this one is the slowest and to ensure that everything is up and running
}
 
uint16_t ldr_read(uint8_t tool){
 ADMUX &= 0xF0;                    //Clear the older channel that was read
 ADMUX |= tool;                //Defines the new ADC channel to be read
 ADCSRA |= (1<<ADSC);                //Starts a new conversion
 while(ADCSRA & (1<<ADSC));            //Wait until the conversion is done
 return ADCW;                    //Returns the ADC value of the chosen channel
}

void print_ldr(uint8_t tool){
 if (tool >=0){
 char buffer[5];
 itoa(ldr_read(tool), buffer, 10);
 printString(buffer);
 printString("\r\n");
 }
}