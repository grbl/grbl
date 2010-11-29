/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
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
#include <avr/sleep.h>
#include <util/delay.h>
#include "stepper.h"
#include "spindle_control.h"
#include "motion_control.h"
#include "gcode.h"
#include "config.h"
#include "wiring_serial.h"
#include "serial_protocol.h"
#include <avr/pgmspace.h>   // contains PSTR definition
//#include "lc_display.h"
#include <WProgram.h>
#include "i2c.h"

extern char buttons[4];
#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1A)

//*************************************************************************************
int main(void)
{
  beginSerial(BAUD_RATE);

//  lcd_init();			// Can use lcd on this arduino, but moved HMI to another 
						// arduino module
  i2c_init();

  // config_reset(); // This routine forces the eeprom config into its default state
  					 // if something really messes it up. Uncomment to use.

  config_init();	// Restore state from eeprom if it is there, else restore default.
  st_init();      // initialize the stepper subsystem
  mc_init();      // initialize motion control subsystem
  spindle_init(); // initialize spindle controller
  gc_init();      // initialize gcode-parser
  sp_init();      // initialize the serial protocol
  
  DDRD |= (1<<3)|(1<<4)|(1<<5);

  for(;;){
    //lcd_report_position();
    i2c_report_position();
    _delay_ms(1);			// Delay is required, otherwise
    i2c_get_buttons();      // i2c_get doesn't work. 1ms seems to be enough
/*    
            printPgmString(PSTR("buttons:\r\n"));  
            printInteger(buttons[0]);
            printPgmString(PSTR(", "));
            printInteger(buttons[1]);
            printPgmString(PSTR(", "));
            printInteger(buttons[2]);
            printPgmString(PSTR(", "));
            printInteger(buttons[3]);
            printPgmString(PSTR("\r\n"));
*/
    
    if (buttons[0]|buttons[1]|buttons[2]|buttons[3]){
    	mc_running=1;
    	STEPPERS_ENABLE_PORT |= (1<<STEPPERS_ENABLE_BIT);
		ENABLE_STEPPER_DRIVER_INTERRUPT();
	}
    
    if (serialAvailable()) sp_process(); // process the serial protocol
    if (mc_in_arc()) mc_continue_arc(); // if busy drawing an arc, keep drawing
  }
  return 0;   /* never reached */
}
