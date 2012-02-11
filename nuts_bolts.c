/*
  nuts_bolts.c - Shared functions
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011-2012 Sungeun K. Jeon  
  
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

#include "nuts_bolts.h"
#include <stdint.h>
#include <stdlib.h>
#include <util/delay.h>

int read_double(char *line, uint8_t *char_counter, double *double_ptr)                  
{
  char *start = line + *char_counter;
  char *end;
  
  *double_ptr = strtod(start, &end);
  if(end == start) { 
    return(false); 
  };

  *char_counter = end - line;
  return(true);
}

// Delays variable defined milliseconds. Compiler compatibility fix for _delay_ms(),
// which only accepts constants in future compiler releases.
void delay_ms(uint16_t ms) 
{
  while ( ms-- ) { _delay_ms(1); }
}

// Delays variable defined microseconds. Compiler compatibility fix for _delay_us(),
// which only accepts constants in future compiler releases.
void delay_us(uint16_t us) 
{
  while ( us-- ) { _delay_us(1); }
}
