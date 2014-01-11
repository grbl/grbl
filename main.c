/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
  Part of Grbl
  
  Copyright (c) 2011-2014 Sungeun K. Jeon
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

#include "system.h"
#include "serial.h"
#include "settings.h"
#include "protocol.h"
#include "gcode.h"
#include "planner.h"
#include "stepper.h"
#include "spindle_control.h"
#include "coolant_control.h"
#include "motion_control.h"
#include "limits.h"
#include "report.h"


// Declare system global variable structure
system_t sys; 


int main(void)
{
  // Initialize system upon power-up.
  serial_init();   // Setup serial baud rate and interrupts
  settings_init(); // Load grbl settings from EEPROM
  stepper_init();  // Configure stepper pins and interrupt timers
  system_init();   // Configure pinout pins and pin-change interrupt
  sei();
  
  memset(&sys, 0, sizeof(sys));  // Clear all system variables
  sys.abort = true;   // Set abort to complete initialization

  // Check for power-up and set system alarm if homing is enabled to force homing cycle
  // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
  // startup scripts, but allows access to settings and internal commands. Only a homing
  // cycle '$H' or kill alarm locks '$X' will disable the alarm.
  // NOTE: The startup script will run after successful completion of the homing cycle, but
  // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
  // things uncontrollably. Very bad.
  #ifdef HOMING_INIT_LOCK
    if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
  #endif
  
  for(;;) {
  
    // Reset the system primary functionality.
    serial_reset_read_buffer(); // Clear serial read buffer
    gc_init(); // Set g-code parser to default state
    spindle_init();
    coolant_init();
    limits_init(); 
    plan_reset(); // Clear block buffer and planner variables
    st_reset(); // Clear stepper subsystem variables.

    // Sync cleared gcode and planner positions to current system position.
    plan_sync_position();
    gc_sync_position();

    // Reset system variables.
    sys.abort = false;
    sys.execute = 0;
    if (bit_istrue(settings.flags,BITFLAG_AUTO_START)) { sys.auto_start = true; }
    else { sys.auto_start = false; }
          
    // Start main loop. Processes inputs and executes them.
    // NOTE: Upon a system abort, the main loop returns and re-initializes the system.      
    protocol_process(); 
    
  }
  return 0;   /* Never reached */
}
