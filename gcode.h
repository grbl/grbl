/*
  gcode.c - rs274/ngc parser.
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

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


#ifndef gcode_h
#define gcode_h
#include <avr/io.h>
#include "stepper.h"

// Initialize the parser
void gc_init();

// Execute one block of rs275/ngc/g-code
uint8_t gc_execute_line(char *line);

#endif
