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

#define LINE_BUFFER_SIZE 128

char line[LINE_BUFFER_SIZE];
uint8_t line_counter;

void prompt() {
  printString("ok\r\n");
  line_counter = 0;
}

void print_result() {
  double position[3];
  int inches_mode;
  uint8_t status_code;
  uint32_t line_number;
  int i; // loop variable
  gc_get_status(position, &status_code, &inches_mode, &line_number);
  printString("\r\n[ ");  
  for(i=X_AXIS; i<=Z_AXIS; i++) {
    printInteger(trunc(position[i]*100));
    printByte(' ');
  }
  printByte(']');
  printByte('@');
  printInteger(line_number);
  printByte(':');
  switch(status_code) {
    case GCSTATUS_OK: printString("0 OK\r\n"); break;
    case GCSTATUS_BAD_NUMBER_FORMAT: printString("1 Bad number format\r\n"); break;
    case GCSTATUS_EXPECTED_COMMAND_LETTER: printString("2 Expected command letter\r\n"); break;
    case GCSTATUS_UNSUPPORTED_STATEMENT: printString("3 Unsupported statement\r\n"); break;
    case GCSTATUS_MOTION_CONTROL_ERROR: printString("4 Motion control error\r\n"); break;
    case GCSTATUS_FLOATING_POINT_ERROR: printString("5 Floating point error\r\n"); break;
  }
}

void sp_init() 
{
  beginSerial(BAUD_RATE);
  
  printString("\r\nGrbl ");
  printString(VERSION);
  printString("\r\n");  
  prompt();
}

void sp_process()
{
  char c;
  while((c = serialRead()) != -1) 
  {
    if((c == '\n')) {  // Line is complete. Then execute!
      line[line_counter] = 0;
      // printString("->");
      // printString(line);
      // printString("<-\r\n");
      gc_execute_line(line);
      line_counter = 0;
      prompt();
    } else if (c <= ' ') { // Throw away whitepace and control characters
    } else if (c >= 'a' && c <= 'z') { // Upcase lowercase
      line[line_counter++] = c-'a'+'A';
    } else {
      line[line_counter++] = c;
    }
  }
}

