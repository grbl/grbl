/*
  interrupt.c - replacement for the avr library of the same name to provide
  dummy register variables

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

#include "interrupt.h"

// dummy register variables
uint16_t timsk0;
uint16_t timsk1;
uint16_t timsk2;
uint16_t ocr1a;
uint16_t ocr2a;
uint16_t tcnt0;
uint16_t tcnt2;
uint16_t tccr0b;
uint16_t tccr2b;
uint16_t tccr1b;
uint16_t tccr0a;
uint16_t tccr1a;
uint16_t tccr2a;
uint16_t pcmsk0;
uint16_t pcicr;

void sei() {};
void cli() {};