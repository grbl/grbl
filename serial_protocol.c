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
#include "motion_control.h"
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

extern int32_t actual_position[3];    // The current actual position of the tool in absolute steps
extern int32_t position[3];    // The current target position of the tool in absolute steps

void sp_init() 
{
  beginSerial(BAUD_RATE);
  printPgmString(PSTR("\r\nGrbl "));
  printPgmString(PSTR(VERSION));
  printPgmString(PSTR("\r\n"));  
  prompt();
}

void return_status(uint8_t status)
{
	switch(status) {
		case GCSTATUS_OK: printPgmString(PSTR("ok\r\n")); break;
		case GCSTATUS_BAD_NUMBER_FORMAT: printPgmString(PSTR("error: bad number format\r\n")); break; 
		case GCSTATUS_EXPECTED_COMMAND_LETTER: printPgmString(PSTR("error: expected command letter\r\n")); break;
		case GCSTATUS_UNSUPPORTED_STATEMENT: printPgmString(PSTR("error: unsupported option number\r\n")); break;
		case GCSTATUS_MOTION_CONTROL_ERROR: printPgmString(PSTR("error: motion control error\r\n")); break;
		case GCSTATUS_FLOATING_POINT_ERROR: printPgmString(PSTR("error: floating point error\r\n")); break;
		case GCSTATUS_UNSUPPORTED_LETTER: printPgmString(PSTR("error: unsupported letter address\r\n")); break;
		default: printPgmString(PSTR("error: unknown\r\n"));
	}
}

void print_count_as_mm(float count)
{
	// Convert position in steps to mm:
	// pos = steps * 1.27/1600 
	//     = steps * .00079375
	//     = steps * 79375 / 100 000 000    (1e-8) 
	
	long whole;
	long fraction;
	float answer;
	
	answer = count*0.79375;
	whole = round(answer/10.0);
	fraction = abs(whole) % 100;
	whole = whole/100;
	if (whole>=0) printPgmString(PSTR(" ")); // Allow space for - if required
	if (abs(whole)<100) printPgmString(PSTR(" "));
	if (abs(whole)<10) printPgmString(PSTR(" "));
	printInteger(whole);
	printPgmString(PSTR("."));
	if (fraction<10) printPgmString(PSTR("0"));
	printInteger(fraction);
} 
 
void sp_report_position()
{

	printPgmString(PSTR("Position:\n\r"));
	printPgmString(PSTR("X: "));
	print_count_as_mm(position[0]);
	printPgmString(PSTR(" : "));
	print_count_as_mm(actual_position[0]);
	printPgmString(PSTR("\n\r"));

	printPgmString(PSTR("Y: "));
	print_count_as_mm(position[1]);
	printPgmString(PSTR(" : "));
	print_count_as_mm(actual_position[1]);
	printPgmString(PSTR("\n\r"));

	printPgmString(PSTR("Z: "));
	print_count_as_mm(position[2]);
	printPgmString(PSTR(" : "));
	print_count_as_mm(actual_position[2]);
	printPgmString(PSTR("\n\r\n\r"));

} 
 
void process_command(char *line)
{
	if (line[1]==0){
		printPgmString(PSTR("You have entered an interpreter command\r\n"));
		printPgmString(PSTR("P\tReturn x, y, z, position\r\n"));
		printPgmString(PSTR("S\tStop current operation"));
	} else {
		switch (line[1]){
			case 'P': sp_report_position(); break;
			case 'S': mc_stop(); break;
			default: printPgmString(PSTR("Unrecognised command in sp_process_command\r\n"));
		}
	}
} 
 
 
void sp_process()
{
  char c;
  uint8_t status;
  
  while((c = serialRead()) != -1) 
  {
  	// Echo sent characters if required:
	serialWrite(c);			// I like to see the characters as they're sent (bob)
	//lcd_print_char(c);
	
	if (c == '\r') {serialWrite('\n');}
	
    if((char_counter > 0) && ((c == '\n') || (c == '\r'))) {  // Line is complete. Then execute!
      line[char_counter] = 0;
      printString(line); printPgmString(PSTR("\r\n"));        
      if (line[0]=='E'){
      	process_command(line);
      	char_counter=0;
      } else {     
      	status = gc_execute_line(line);
      	char_counter = 0; 
      	return_status(status);
      }
      prompt();
    } else if (c <= ' ') { // Throw away whitepace and control characters
    } else if (c >= 'a' && c <= 'z') { // Upcase lowercase
      line[char_counter++] = c-'a'+'A';
    } else {
      line[char_counter++] = c;
    }
  }
}

