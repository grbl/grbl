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

#include <inttypes.h>

// dummy register variables
extern uint16_t timsk0;
extern uint16_t timsk1;
extern uint16_t timsk2;
extern uint16_t tcnt0;
extern uint16_t tcnt2;
extern uint16_t tccr0b;
extern uint16_t tccr0a;
extern uint16_t tccr2a;
extern uint16_t tccr2b;
extern uint16_t tccr1b;
extern uint16_t tccr1a;
extern uint16_t ocr1a;
extern uint16_t ocr2a;
extern uint16_t pcmsk0;
extern uint16_t pcicr;

// macros to turn avr interrupts into regular functions
#define TIMER1_COMPA_vect
#define ISR(a) void interrupt_ ## a ()

// enable interrupts does nothing in the simulation environment
void sei();
void cli();

// dummy macros for interrupt related registers
#define TIMSK0 timsk0
#define TIMSK1 timsk1
#define TIMSK2 timsk2
#define OCR1A ocr1a
#define OCR2A ocr2a
#define OCIE1A 0
#define OCIE2A 0
#define TCNT0 tcnt0
#define TCNT2 tcnt2
#define TCCR0B tccr0b
#define TCCR0A tccr0a
#define TCCR1A tccr1a
#define TCCR1B tccr1b
#define TCCR2A tccr2a
#define TCCR2B tccr2b
#define CS21 0
#define CS10 0
#define WGM13 0
#define WGM12 0
#define WGM11 0
#define WGM10 0
#define WGM21 0
#define COM1A0 0
#define COM1B0 0
#define TOIE0 0
#define TOIE2 0
#define PCICR pcicr

#endif
