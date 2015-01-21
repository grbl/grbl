/*
  ldr.h - analog read methods
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

#ifndef laser_control_h
#define laser_control_h 

#include <avr/io.h>
#include <stdlib.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

void ldr_init(void);
uint16_t ldr_read(uint8_t channel);

#endif