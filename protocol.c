/*
  protocol.c - the serial protocol master control unit
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

#include <avr/io.h>
#include "protocol.h"
#include "gcode.h"
#include "wiring_serial.h"
#include "settings.h"
#include "config.h"
#include <math.h>
#include "nuts_bolts.h"
#include <avr/pgmspace.h>
#define LINE_BUFFER_SIZE 50

static char line[LINE_BUFFER_SIZE];
static uint8_t char_counter;

static void status_message(int status_code) {
  if (status_code == 0) {
    printPgmString(PSTR("ok\r\n"));
  } else {
    printPgmString(PSTR("error: "));
    switch(status_code) {          
      case STATUS_BAD_NUMBER_FORMAT:
      printPgmString(PSTR("Bad number format\r\n")); break;
      case STATUS_EXPECTED_COMMAND_LETTER:
      printPgmString(PSTR("Expected command letter\r\n")); break;
      case STATUS_UNSUPPORTED_STATEMENT:
      printPgmString(PSTR("Unsupported statement\r\n")); break;
      case STATUS_FLOATING_POINT_ERROR:
      printPgmString(PSTR("Floating point error\r\n")); break;
      default:
      printInteger(status_code);
      printPgmString(PSTR("\r\n"));
    }
  }
}

void protocol_init() 
{
  beginSerial(BAUD_RATE);  
  printPgmString(PSTR("\r\nGrbl " GRBL_VERSION));
  printPgmString(PSTR("\r\n"));  
}

// Executes one line of input according to protocol
uint8_t protocol_execute_line(char *line) {
  if(line[0] == '$') {
    return(settings_execute_line(line)); // Delegate lines starting with '$' to the settings module
  } else {
    return(gc_execute_line(line));    // Everything else is gcode
  }
}

void protocol_process()
{
  char c;
  while((c = serialRead()) != -1) 
  {
    if((char_counter > 0) && ((c == '\n') || (c == '\r'))) {  // Line is complete. Then execute!
      line[char_counter] = 0; // treminate string
      status_message(protocol_execute_line(line));
      char_counter = 0; // reset line buffer index
    } else if (c <= ' ') { // Throw away whitepace and control characters
    } else if (c >= 'a' && c <= 'z') { // Upcase lowercase
      line[char_counter++] = c-'a'+'A';
    } else {
      line[char_counter++] = c;
    }
  }
}
