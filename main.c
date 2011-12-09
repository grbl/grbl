/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Sungeun K. Jeon
  Copyright (c) 2011 Jens Geisler
  
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
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "config.h"
#include "planner.h"
#include "nuts_bolts.h"
#include "stepper.h"
#include "spindle_control.h"
#include "motion_control.h"
#include "gcode.h"
#include "protocol.h"
#include "limits.h"

#include "settings.h"
#include "serial.h"

#include "print.h"

// Declare system global variables
uint8_t sys_abort; // Global system abort flag
volatile uint8_t sys_state; // Global system state variable

int main(void)
{
  // Initialize system
  sei(); // Enable interrupts
  serial_init(BAUD_RATE); // Setup serial baud rate and interrupts
  st_init(); // Setup stepper pins and interrupt timers
  sys_abort = true;   // Set abort to complete initialization
                    
  while(1) {
  
    // Upon a system abort, the main program will return to this loop. Once here, it is safe to 
    // re-initialize the system. Upon startup, the system will automatically reset to finish the 
    // initialization process.
    if (sys_abort) {
      // Execute system reset
      sys_state = 0; // Reset system state
      sys_abort = false; // Release system abort
        
      // Reset system.
      serial_reset_read_buffer(); // Clear serial read buffer
      settings_init(); // Load grbl settings from EEPROM
      protocol_init(); // Clear incoming line data
      plan_init(); // Clear block buffer and planner variables
      gc_init(); // Set g-code parser to default state
      spindle_init();   
      limits_init();

      // TODO: For now, the stepper subsystem tracks the absolute stepper position from the point
      // of power up or hard reset. This reset is a soft reset, where the information of the current
      // position is not lost after a system abort. This is not guaranteed to be correct, since 
      // during an abort, the steppers can lose steps in the immediate stop. However, if a feed
      // hold is performed before a system abort, this position should be correct. In the next few
      // updates, this soft reset feature will be fleshed out along with the status reporting and
      // jogging features.
      st_reset(); // Clear stepper subsystem variables. Machine position variable is not reset.

      // Print grbl initialization message
      printPgmString(PSTR("\r\nGrbl " GRBL_VERSION));
      printPgmString(PSTR("\r\n'$' to dump current settings\r\n"));
    }
    
    protocol_execute_runtime();
    protocol_process(); // ... process the serial protocol
    
  }
  return 0;   /* never reached */
}
