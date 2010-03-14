/*
  serial_protocol.c - the serial protocol master control unit
  Part of Grbl

  Copyright (c) 2009 Simen Svale Skogsrud

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

#include <avr/io.h>
#include "serial_protocol.h"
#include "gcode.h"
#include "wiring_serial.h"
#include "config.h"
#include <math.h>
#include "nuts_bolts.h"
#include <avr/pgmspace.h>
#define LINE_BUFFER_SIZE 50

char line[LINE_BUFFER_SIZE];
uint8_t char_counter;

void prompt() {
  printPgmString(PSTR("ok\r\n"));
}

void sp_init() 
{
  beginSerial(BAUD_RATE);
  
  printPgmString(PSTR("\r\nGrbl "));
  printPgmString(PSTR(VERSION));
  printPgmString(PSTR("\r\n"));  
  prompt();
}

void sp_process()
{
  char c;
  while((c = serialRead()) != -1) 
  {
    if((c == '\n')) {  // Line is complete. Then execute!
      line[char_counter] = 0;
      printString(line); printPgmString(PSTR("\r\n"));        
      gc_execute_line(line);
      char_counter = 0;
      prompt();
    } else if (c <= ' ') { // Throw away whitepace and control characters
    } else if (c >= 'a' && c <= 'z') { // Upcase lowercase
      line[char_counter++] = c-'a'+'A';
    } else {
      line[char_counter++] = c;
    }
  }
}
