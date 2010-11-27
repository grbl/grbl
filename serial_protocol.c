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
#include "stepper.h"
#include "wiring_serial.h"
#include "config.h"
#include <math.h>
#include "nuts_bolts.h"
#include <avr/pgmspace.h>

#define LINE_BUFFER_SIZE 50

char line[LINE_BUFFER_SIZE];

char Verbose=FALSE;

uint8_t char_counter;

void prompt() {
  printPgmString(PSTR("ok\r\n"));
}

extern int32_t actual_position[3];    // The current actual position of the tool in absolute steps
extern int32_t position[3];    // The current target position of the tool in absolute steps

void sp_init() 
{
  // beginSerial(BAUD_RATE);
/*  
  printPgmString(PSTR("PS2 controller init result: "));
  printInteger(ps2_init());
  printPgmString(PSTR("\n\r"));
*/  
/*  printPgmString(PSTR("12345678901234567890123456789012345678\n\r"));
  printPgmString(PSTR("12345678901234567890123456789012345678\n\r"));
  printPgmString(PSTR("12345678901234567890123456789012345678\n\r"));
  printPgmString(PSTR("12345678901234567890123456789012345678\n\r"));
*/
  printPgmString(PSTR("\r\nGrbl "));
  printPgmString(PSTR(VERSION));
  printPgmString(PSTR("\r\n"));  
  prompt();
}

void return_status(uint8_t status)
{
	if (Verbose){
		switch(status) {
			case GCSTATUS_OK: printPgmString(PSTR("ok\r\n")); break;
			case GCSTATUS_BAD_NUMBER_FORMAT: printPgmString(PSTR("error: bad number format\r\n")); break; 
			case GCSTATUS_EXPECTED_COMMAND_LETTER: printPgmString(PSTR("error: expected command letter\r\n")); break;
			case GCSTATUS_UNSUPPORTED_STATEMENT: printPgmString(PSTR("error: unsupported option number\r\n")); break;
			case GCSTATUS_MOTION_CONTROL_ERROR: printPgmString(PSTR("error: motion control error\r\n")); break;
			case GCSTATUS_FLOATING_POINT_ERROR: printPgmString(PSTR("error: floating point error\r\n")); break;
			case GCSTATUS_UNSUPPORTED_LETTER: printPgmString(PSTR("error: unsupported letter address\r\n")); break;
			case GCSTATUS_BUFFER_FULL: printPgmString(PSTR("error: stepper buffer is full\r\n")); break;
			default: printPgmString(PSTR("error: unknown\r\n"));
		}
	} else {
		if (status == GCSTATUS_OK){
			printPgmString(PSTR("ok\r\n"));
		} else {
			printPgmString(PSTR("err\r\n"));
		}
	}
}

void print_count_as_mm(float count, char Pad)
{
	// Convert position in steps to mm:
	// pos = steps * 1.27/1600 
	//     = steps * .00079375
	//     = steps * 79375 / 100 000 000    (1e-8) 
	
	long whole;
	long fraction;
	float answer;
	
	answer = count*DEFAULT_UM_PER_STEP;
	whole = round(answer/10.0);
	fraction = labs(whole) % 100;			// must be labs, otherwise overflows at 327.67 mm
											// and gets fractional part wrong higher than that.
	whole = whole/100;
	if ((whole>=0) &&(Pad)) printPgmString(PSTR(" ")); // Allow space for - if required
	if ((abs(whole)<100) &&(Pad)) printPgmString(PSTR(" "));
	if (abs(whole)<10) {
		if ((whole==0) & (answer<0)){		// Needed otherwise values between -0.99 and 0 come
			printPgmString(PSTR("-"));		// out without a minus sign.
		} else {
			if (Pad) printPgmString(PSTR(" "));
		}
	}
	printInteger(whole);
	if ((Pad)||(fraction!=0)){
	    printPgmString(PSTR("."));
	    if (fraction<10) printPgmString(PSTR("0"));
	    printInteger(fraction);
	}
} 
 
void sp_report_position()
{

	printPgmString(PSTR("Position:\n\r"));
	printPgmString(PSTR("X: "));
	print_count_as_mm(position[0], 1);
	printPgmString(PSTR(" : "));
	print_count_as_mm(actual_position[0],1);
	printPgmString(PSTR("\n\r"));

	printPgmString(PSTR("Y: "));
	print_count_as_mm(position[1],1);
	printPgmString(PSTR(" : "));
	print_count_as_mm(actual_position[1],1);
	printPgmString(PSTR("\n\r"));

	printPgmString(PSTR("Z: "));
	print_count_as_mm(position[2],1);
	printPgmString(PSTR(" : "));
	print_count_as_mm(actual_position[2],1);
	printPgmString(PSTR("\n\r\n\r"));

} 

void sp_quick_position()
{
	// Byte 1: steppers are running under 'M'anual control, 'A'uto control, or are 'O'ff
    if (mc_running) {
    	if (buttons_in_use) printPgmString(PSTR("M")); else printPgmString(PSTR("A"));
    } else {
    	printPgmString(PSTR("O"));
    }
	// Byte 2: stepper buffer is 'F'ull or 'R'eady
    if (st_buffer_full()) printPgmString(PSTR("F")); else printPgmString(PSTR("R"));
    printPgmString(PSTR("N"));
	printInteger(acting_line_number);
	printPgmString(PSTR("X"));
	print_count_as_mm(actual_position[0], 0);
	printPgmString(PSTR("Y"));
	print_count_as_mm(actual_position[1], 0);
	printPgmString(PSTR("Z"));
	print_count_as_mm(actual_position[2], 0);
	printPgmString(PSTR("\n\r"));

} 

extern char buttons[4];

void sp_report_buttons()
{
	printPgmString(PSTR("Buttons:"));
	printInteger(buttons[0]);
	printPgmString(PSTR(","));
	printInteger(buttons[1]);
	printPgmString(PSTR(","));
	printInteger(buttons[2]);
	printPgmString(PSTR(","));
	printInteger(buttons[3]);
	printPgmString(PSTR("\n\r"));
}
 
void process_command(char *line)
{
	if (line[1]==0){
		printPgmString(PSTR("You have entered an interpreter command\r\n"));
		printPgmString(PSTR("B\tReport button values on HMI\r\n"));
		
		printPgmString(PSTR("P\tReturn x, y, z, position\r\n"));
		printPgmString(PSTR("Q\tQuick response: running mode (O - off, M-anual, A-uto), \r\n"));
		printPgmString(PSTR( "\t\t\t\t\tbuffer ready (F-ull or R-eady),\r\n"));
        printPgmString(PSTR( "\t\t\t\t\tline number,\r\n"));
        printPgmString(PSTR( "\t\t\t\t\tx, y, z, position\r\n"));
		printPgmString(PSTR("S\tStop current operation\r\n"));
		printPgmString(PSTR("T\tPut communications into Terse mode (default on startup)\r\n"));
		printPgmString(PSTR("V\tPut communications into Verbose mode, also enables echo\r\n"));
		
		printPgmString(PSTR("\n\r"));
	
	} else {
		switch (line[1]){
			case 'B': sp_report_buttons(); break;
			case 'Q': sp_quick_position(); break;
			case 'P': sp_report_position(); break;
			case 'S': mc_stop(); break;
			case 'T': Verbose = FALSE; break;
			case 'V': Verbose = TRUE; break;
			default: printPgmString(PSTR("Unrecognised command in sp_process_command\r\n"));
		}
		if (line[1]!='Q') prompt();
	}
} 
/* 
void  ps2_process()
{
	ps2_read_gamepad();
  if(ps2_ButtonPressed(PSB_RED))             //will be TRUE if button was JUST pressed
       printPgmString(PSTR("Circle just pressed\n\r"));
       
  if(ps2_ButtonReleased(PSB_PINK))             //will be TRUE if button was JUST released
       printPgmString(PSTR("Square just released\n\r"));     
  
  if(ps2_NewButtonState(PSB_BLUE))            //will be TRUE if button was JUST pressed OR released
       printPgmString(PSTR("X just changed\n\r"));           
}
*/
 
/* 
void gc_machine(char *line)
{ 
  uint8_t status;
  if (line[0]=='E'){
	  if (mc_in_arc()){
			mc_arc_to_line();
		} else {
		   process_command(line);
		}
		char_counter=0;
  } else {   
		if (mc_in_arc()){
			mc_arc_to_line();
		} else {
			status = gc_execute_line(line);
		}
		char_counter = 0; 
		return_status(status);
  }
} 
*/
void sp_process()
{
  char c;
  uint8_t status;

// Only gets processed if there is something waiting on the serial port:
  while((c = serialRead()) != -1) 
  {
  	// Echo sent characters if required:
  	if (Verbose) serialWrite(c);		
		
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
    } else if (c <= ' ') { // Throw away whitepace and control characters
    } else if (c >= 'a' && c <= 'z') { // Upcase lowercase
      line[char_counter++] = c-'a'+'A';
    } else {
      line[char_counter++] = c;
    }
  }
}

