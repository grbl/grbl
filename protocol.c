/*
  protocol.c - the serial protocol master control unit
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011-2012 Sungeun K. Jeon  

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
#include "serial.h"
#include "print.h"
#include "settings.h"
#include "config.h"
#include <math.h>
#include "nuts_bolts.h"
#include <avr/pgmspace.h>
#include "stepper.h"
#include "planner.h"

static char line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated.
static uint8_t char_counter; // Last character counter in line variable.
static uint8_t iscomment; // Comment/block delete flag for processor to ignore comment characters.

// Method to handle status messages, from an 'ok' to report any 'error' that has occurred.
// Errors can originate from the g-code parser, settings module, or a critical error, such
// as a triggered hard limit.
void protocol_status_message(uint8_t status_code) 
{
  // TODO: Compile time option to only return numeric codes for GUIs.
  if (status_code == 0) { // STATUS_OK
    printPgmString(PSTR("ok\r\n"));
  } else {
    printPgmString(PSTR("error: "));
    switch(status_code) {          
      case STATUS_BAD_NUMBER_FORMAT:
      printPgmString(PSTR("Bad number format")); break;
      case STATUS_EXPECTED_COMMAND_LETTER:
      printPgmString(PSTR("Expected command letter")); break;
      case STATUS_UNSUPPORTED_STATEMENT:
      printPgmString(PSTR("Unsupported statement")); break;
      case STATUS_FLOATING_POINT_ERROR:
      printPgmString(PSTR("Floating point error")); break;
      case STATUS_MODAL_GROUP_VIOLATION:
      printPgmString(PSTR("Modal group violation")); break;
      case STATUS_INVALID_STATEMENT:
      printPgmString(PSTR("Invalid statement")); break;
      case STATUS_HARD_LIMIT:
      printPgmString(PSTR("Limit triggered")); break;
      case STATUS_SETTING_DISABLED:
      printPgmString(PSTR("Grbl setting disabled")); break;
      case STATUS_SETTING_STEPS_NEG:
      printPgmString(PSTR("Steps/mm must be > 0.0")); break;
      case STATUS_SETTING_STEP_PULSE_MIN:
      printPgmString(PSTR("Step pulse must be >= 3 microseconds")); break;
      case STATUS_SETTING_READ_FAIL:
      printPgmString(PSTR("Failed to read EEPROM settings. Using defaults")); break;
    }
    printPgmString(PSTR("\r\n"));
  }
}


// Prints miscellaneous messages. This serves as a centralized method to provide additional
// user feedback for things that do not pass through the protocol_execute_line() function
// and are not errors or confirmations, such as setup warnings and how to exit alarms.
void protocol_misc_message(uint8_t message_code)
{
  // TODO: Install silence misc messages option in settings
  switch(message_code) {
    case MESSAGE_SYSTEM_ALARM:
    printPgmString(PSTR("<ALARM: Reset to continue>")); break;
    case MESSAGE_HOMING_ENABLE:
    printPgmString(PSTR("warning: Install all axes limit switches before use")); break;
  }
  printPgmString(PSTR("\r\n"));
}


void protocol_status_report()
{
 // TODO: Status report data is written to the user here. This function should be able to grab a 
 // real-time snapshot of the stepper subprogram and the actual location of the CNC machine. At a
 // minimum, status report should return real-time location information. Other important information
 // may be distance to go on block, processed block id, and feed rate. A secondary, non-critical
 // status report may include g-code state, i.e. inch mode, plane mode, absolute mode, etc. 
 //   The report generated must be as short as possible, yet still provide the user easily readable
 // information, i.e. '[0.23,120.4,2.4]'. This is necessary as it minimizes the computational 
 // overhead and allows grbl to keep running smoothly, especially with g-code programs with fast, 
 // short line segments and interface setups that require real-time status reports (5-20Hz).

 // **Under construction** Bare-bones status report. Provides real-time machine position relative to 
 // the system power on location (0,0,0) and work coordinate position (G54 and G92 applied).
 // The following are still needed: user setting of output units (mm|inch), compressed (non-human 
 // readable) data for interfaces?, save last known position in EEPROM?, code optimizations, solidify
 // the reporting schemes, move to a separate .c file for easy user accessibility, and setting the
 // home position by the user (likely through '$' setting interface).
 // Successfully tested at a query rate of 10-20Hz while running a gauntlet of programs at various 
 // speeds.
 uint8_t i;
 int32_t current_position[3]; // Copy current state of the system position variable
 memcpy(current_position,sys.position,sizeof(sys.position));
 float print_position[3];

 // Report machine position
 printPgmString(PSTR("MPos:[")); 
 for (i=0; i<= 2; i++) {
   print_position[i] = current_position[i]/settings.steps_per_mm[i];
   if (bit_istrue(settings.flags,BITFLAG_REPORT_INCHES)) { print_position[i] *= INCH_PER_MM; }
   printFloat(print_position[i]);
   if (i < 2) { printPgmString(PSTR(",")); }
   else { printPgmString(PSTR("]")); }
 }
 
 // Report work position
 printPgmString(PSTR(",WPos:[")); 
 for (i=0; i<= 2; i++) {
   if (bit_istrue(settings.flags,BITFLAG_REPORT_INCHES)) {
     print_position[i] -= (sys.coord_system[sys.coord_select][i]+sys.coord_offset[i])*INCH_PER_MM;
   } else {
     print_position[i] -= sys.coord_system[sys.coord_select][i]+sys.coord_offset[i];
   }
   printFloat(print_position[i]);
   if (i < 2) { printPgmString(PSTR(",")); }
   else { printPgmString(PSTR("]")); }
 }
   
 printPgmString(PSTR("\r\n"));
}


void protocol_init() 
{
  // Print grbl initialization message
  printPgmString(PSTR("\r\nGrbl " GRBL_VERSION));
  printPgmString(PSTR("\r\n'$' to dump current settings\r\n"));

  char_counter = 0; // Reset line input
  iscomment = false;
}


// Executes run-time commands, when required. This is called from various check points in the main
// program, primarily where there may be a while loop waiting for a buffer to clear space or any
// point where the execution time from the last check point may be more than a fraction of a second.
// This is a way to execute runtime commands asynchronously (aka multitasking) with grbl's g-code
// parsing and planning functions. This function also serves as an interface for the interrupts to 
// set the system runtime flags, where only the main program handles them, removing the need to
// define more computationally-expensive volatile variables.
// NOTE: The sys.execute variable flags are set by the serial read subprogram, except where noted.
void protocol_execute_runtime()
{
  if (sys.execute) { // Enter only if any bit flag is true
    uint8_t rt_exec = sys.execute; // Avoid calling volatile multiple times
    
    // System alarm. Something has gone wrong. Disable everything until system reset.
    if (rt_exec & EXEC_ALARM) {
      if (bit_isfalse(sys.execute,EXEC_RESET)) { protocol_misc_message(MESSAGE_SYSTEM_ALARM); }
      while (bit_isfalse(sys.execute,EXEC_RESET)) { sleep_mode(); }
      bit_false(sys.execute,EXEC_ALARM);
    } 
  
    // System abort. Steppers have already been force stopped.
    if (rt_exec & EXEC_RESET) {
      sys.abort = true; 
      return; // Nothing else to do but exit.
    }
    
    // Execute and serial print status
    if (rt_exec & EXEC_STATUS_REPORT) { 
      protocol_status_report();
      bit_false(sys.execute,EXEC_STATUS_REPORT);
    }
    
    // Initiate stepper feed hold
    if (rt_exec & EXEC_FEED_HOLD) {
      st_feed_hold(); // Initiate feed hold.
      bit_false(sys.execute,EXEC_FEED_HOLD);
    }
    
    // Reinitializes the stepper module running flags and re-plans the buffer after a feed hold.
    // NOTE: EXEC_CYCLE_STOP is set by the stepper subsystem when a cycle or feed hold completes.
    if (rt_exec & EXEC_CYCLE_STOP) {
      st_cycle_reinitialize();
      bit_false(sys.execute,EXEC_CYCLE_STOP);
    }
    
    if (rt_exec & EXEC_CYCLE_START) { 
      st_cycle_start(); // Issue cycle start command to stepper subsystem
      if (bit_istrue(settings.flags,BITFLAG_AUTO_START)) {
        sys.auto_start = true; // Re-enable auto start after feed hold.
      }
      bit_false(sys.execute,EXEC_CYCLE_START);
    }
  }  
}  


// Executes one line of input according to protocol
uint8_t protocol_execute_line(char *line) 
{     
  if(line[0] == '$') {
  
    // TODO: Re-write this '$' as a way to change runtime settings without having to reset, i.e.
    // auto-starting, status query output formatting and type, jog mode (axes, direction, and
    // nominal feedrate), toggle block delete, etc. This differs from the EEPROM settings, as they
    // are considered defaults and loaded upon startup/reset.
    //   This use is envisioned where '$' itself dumps settings and help. Defined characters
    // proceeding the '$' may be used to setup modes, such as jog mode with a '$J=X100' for X-axis
    // motion with a nominal feedrate of 100mm/min. Writing EEPROM settings will likely stay the 
    // same or similar. Should be worked out in upcoming releases.    
    return(settings_execute_line(line)); // Delegate lines starting with '$' to the settings module

  // } else if { 
  //
  // JOG MODE
  //
  // TODO: Here jogging can be placed for execution as a seperate subprogram. It does not need to be 
  // susceptible to other runtime commands except for e-stop. The jogging function is intended to
  // be a basic toggle on/off with controlled acceleration and deceleration to prevent skipped 
  // steps. The user would supply the desired feedrate, axis to move, and direction. Toggle on would
  // start motion and toggle off would initiate a deceleration to stop. One could 'feather' the
  // motion by repeatedly toggling to slow the motion to the desired location. Location data would 
  // need to be updated real-time and supplied to the user through status queries.
  //   More controlled exact motions can be taken care of by inputting G0 or G1 commands, which are 
  // handled by the planner. It would be possible for the jog subprogram to insert blocks into the
  // block buffer without having the planner plan them. It would need to manage de/ac-celerations 
  // on its own carefully. This approach could be effective and possibly size/memory efficient.

  } else {
    return(gc_execute_line(line));    // Everything else is gcode
    // TODO: Install option to set system alarm upon any error code received back from the
    // the g-code parser. This is a common safety feature on CNCs to help prevent crashes
    // if the g-code doesn't perform as intended.
  }
}


// Process one line of incoming serial data. Remove unneeded characters and capitalize.
void protocol_process()
{
  uint8_t c;
  while((c = serial_read()) != SERIAL_NO_DATA) {
    if ((c == '\n') || (c == '\r')) { // End of line reached

      // Runtime command check point before executing line. Prevent any furthur line executions.
      // NOTE: If there is no line, this function should quickly return to the main program when
      // the buffer empties of non-executable data.
      protocol_execute_runtime();
      if (sys.abort) { return; } // Bail to main program upon system abort    

      if (char_counter > 0) {// Line is complete. Then execute!
        line[char_counter] = 0; // Terminate string
        protocol_status_message(protocol_execute_line(line));
      } else { 
        // Empty or comment line. Skip block.
        protocol_status_message(STATUS_OK); // Send status message for syncing purposes.
      }
      char_counter = 0; // Reset line buffer index
      iscomment = false; // Reset comment flag
      
    } else {
      if (iscomment) {
        // Throw away all comment characters
        if (c == ')') {
          // End of comment. Resume line.
          iscomment = false;
        }
      } else {
        if (c <= ' ') { 
          // Throw away whitepace and control characters
        } else if (c == '/') {
          // Disable block delete and throw away characters. Will ignore until EOL.
          #if BLOCK_DELETE_ENABLE
            iscomment = true;
          #endif
        } else if (c == '(') {
          // Enable comments flag and ignore all characters until ')' or EOL.
          iscomment = true;
        } else if (char_counter >= LINE_BUFFER_SIZE-1) {
          // Throw away any characters beyond the end of the line buffer          
        } else if (c >= 'a' && c <= 'z') { // Upcase lowercase
          line[char_counter++] = c-'a'+'A';
        } else {
          line[char_counter++] = c;
        }
      }
    }
  }
}
