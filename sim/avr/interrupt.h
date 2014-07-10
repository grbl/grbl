/*
  interrupt.h - replacement for the avr include of the same name to provide
  dummy register variables and macros

  Part of Grbl Simulator

  Copyright (c) 2012 Jens Geisler

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef interrupt_h
#define interrupt_h

// macros to turn avr interrupts into regular functions
//#define TIMER1_COMPA_vect
#define ISR(a) void interrupt_ ## a ()

// Stubs of the hardware interrupt functions we are using
void interrupt_TIMER0_COMPA_vect();
void interrupt_TIMER1_COMPA_vect();
void interrupt_TIMER0_OVF_vect();
void interrupt_SERIAL_UDRE();
void interrupt_SERIAL_RX();
void interrupt_LIMIT_INT_vect();
void interrupt_WDT_vect();


//pseudo-Interrupt vector table  
typedef void(*isr_fp)(void);
extern isr_fp compa_vect[6];
extern isr_fp compb_vect[6];
extern isr_fp ovf_vect[6];
extern isr_fp wdt_vect;
extern isr_fp pc_vect; //pin change

// enable interrupts now does something in the simulation environment
#define SEI 0x80
void sei();
void cli();

//simulate timer operation
void timer_interrupts();


#endif
