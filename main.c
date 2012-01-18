/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Sungeun K. Jeon

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

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
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

// Declare system global variable structure
system_t sys; 

int main(void)
{
  // Initialize system
  serial_init(BAUD_RATE); // Setup serial baud rate and interrupts
  st_init(); // Setup stepper pins and interrupt timers
  sei(); // Enable interrupts

  memset(&sys, 0, sizeof(sys));  // Clear all system variables
  sys.abort = true;   // Set abort to complete initialization
                    
  while(1) {
  
    // Execute system reset upon a system abort, where the main program will return to this loop.
    // Once here, it is safe to re-initialize the system. At startup, the system will automatically
    // reset to finish the initialization process.
    if (sys.abort) {
      
      // Retain last known machine position. If the system abort occurred while in motion, machine
      // position is not guaranteed, since a hard stop can cause the steppers to lose steps. Always
      // perform a feedhold before an abort, if maintaining accurate machine position is required.
      int32_t last_position[3];
      memcpy(last_position, sys.position, sizeof(sys.position)); // last_position[] = sys.position[]
      
      // Clear all system variables
      memset(&sys, 0, sizeof(sys));
      
      // Update last known machine position. Set the post-abort work position as the origin [0,0,0],
      // which corresponds to the g-code parser and planner positions after re-initialization.
      memcpy(sys.position, last_position, sizeof(last_position)); // sys.position[] = last_position[]
      memcpy(sys.coord_offset, last_position, sizeof(last_position)); // sys.coord_offset[] = last_position[]
      
      // Reset system.
      serial_reset_read_buffer(); // Clear serial read buffer
      settings_init(); // Load grbl settings from EEPROM
      protocol_init(); // Clear incoming line data
      plan_init(); // Clear block buffer and planner variables
      gc_init(); // Set g-code parser to default state
      spindle_init();   
      limits_init();
      st_reset(); // Clear stepper subsystem variables.
      
      // Set system runtime defaults
      // TODO: Eventual move to EEPROM from config.h when all of the new settings are worked out. 
      // Mainly to avoid having to maintain several different versions.
      #ifdef CYCLE_AUTO_START
        sys.auto_start = true;
      #endif
    }
    
    protocol_execute_runtime();
    protocol_process(); // ... process the serial protocol
    
  }
  return 0;   /* never reached */
}
