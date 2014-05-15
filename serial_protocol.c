/*
  serial_protocol.c - the serial protocol master control unit
  Part of Grbl

  The MIT License (MIT)

  Grbl(tm) - Embedded CNC g-code interpreter and motion-controller
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#include <avr/io.h>
#include "serial_protocol.h"
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

void status_message(int status_code) {
  switch(status_code) {          
    case GCSTATUS_OK:
    printPgmString(PSTR("ok\n\r")); break;
    case GCSTATUS_BAD_NUMBER_FORMAT:
    printPgmString(PSTR("error: Bad number format\n\r")); break;
    case GCSTATUS_EXPECTED_COMMAND_LETTER:
    printPgmString(PSTR("error: Expected command letter\n\r")); break;
    case GCSTATUS_UNSUPPORTED_STATEMENT:
    printPgmString(PSTR("error: Unsupported statement\n\r")); break;
    case GCSTATUS_FLOATING_POINT_ERROR:
    printPgmString(PSTR("error: Floating point error\n\r")); break;
    default:
    printPgmString(PSTR("error: "));
    printInteger(status_code);
    printPgmString(PSTR("\n\r"));
  }
}

void sp_init() 
{
  beginSerial(BAUD_RATE);  
  printPgmString(PSTR("\r\nGrbl " GRBL_VERSION));
  printPgmString(PSTR("\r\n"));  
}

void sp_process()
{
  char c;
  while((c = serialRead()) != -1) 
  {
    if((char_counter > 0) && ((c == '\n') || (c == '\r'))) {  // Line is complete. Then execute!
      line[char_counter] = 0; // treminate string
      status_message(gc_execute_line(line));
      char_counter = 0; // reset line buffer index
    } else if (c <= ' ') { // Throw away whitepace and control characters
    } else if (c >= 'a' && c <= 'z') { // Upcase lowercase
      line[char_counter++] = c-'a'+'A';
    } else {
      line[char_counter++] = c;
    }
  }
}
