/*
  protocol.c - the serial protocol master control unit
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

#define LINE_BUFFER_SIZE 50

static char line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated.
static uint8_t char_counter; // Last character counter in line variable.
static uint8_t iscomment; // Comment/block delete flag for processor to ignore comment characters.

static void status_message(int status_code) 
{
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


void protocol_status_report()
{
 // TODO: Status report data is written to the user here. This function should be able to grab a 
 // real-time snapshot of the stepper subprogram and the actual location of the CNC machine. At a
 // minimum, status report should return real-time location information. Other important information
 // may be distance to go on block, processed block id, and feed rate. A secondary, non-critical
 // status report may include g-code state, i.e. inch mode, plane mode, absolute mode, etc. 
 //   The report generated must be as short as possible, yet still provide the user easily readable
 // information, i.e. 'x0.23 y120.4 z2.4'. This is necessary as it minimizes the computational 
 // overhead and allows grbl to keep running smoothly, especially with g-code programs with fast, 
 // short line segments and interface setups that require real-time status reports (10-20Hz).
 printString("Query Received.\r\n"); // Notify that it's working.
}


void protocol_init() 
{
  char_counter = 0; // Reset line input
  iscomment = false;
}


// Executes run-time commands, when required. This is called from various check points in the main
// program, primarily where there may be a while loop waiting for a buffer to clear space or any
// point where the execution time from the last check point may be more than a fraction of a second.
// This is a way to execute runtime commands asynchronously (aka multitasking) with grbl's g-code
// parsing and planning functions.
// NOTE: The sys_state variable flags are set by the serial read subprogram, except where noted.
void protocol_execute_runtime()
{
  if (sys_state) { // Enter only if any bit flag is enabled
  
    // System abort. Steppers have already been force stopped.
    if (sys_state & BIT_RESET) {
      sys_abort = true; 
      return; // Nothing else to do but exit.
    }
    
    // Execute and serial print status
    if (sys_state & BIT_STATUS_REPORT) { 
      protocol_status_report();
      sys_state ^= BIT_STATUS_REPORT; // Toggle off
    }
    
    // Initiate stepper feed hold
    if (sys_state & BIT_FEED_HOLD) {
      st_feed_hold();
      sys_state ^= BIT_FEED_HOLD; // Toggle off   
    }
    
    // Re-plans the buffer after a feed hold completes
    // NOTE: BIT_REPLAN_CYCLE is set by the stepper subsystem when the feed hold is complete.
    if (sys_state & BIT_REPLAN_CYCLE) {
      st_cycle_reinitialize();
      sys_state ^= BIT_REPLAN_CYCLE; // Toggle off
    }
    
    if (sys_state & BIT_CYCLE_START) { 
      st_cycle_start(); // Issue cycle start command to stepper subsystem
      sys_state ^= BIT_CYCLE_START; // Toggle off
    } 
  }
}  


// Executes one line of input according to protocol
uint8_t protocol_execute_line(char *line) 
{     
  if(line[0] == '$') {
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
      if (sys_abort) { return; } // Bail to main program upon system abort    

      if (char_counter > 0) {// Line is complete. Then execute!
        line[char_counter] = 0; // Terminate string
        status_message(protocol_execute_line(line));
      } else { 
        // Empty or comment line. Skip block.
        status_message(STATUS_OK); // Send status message for syncing purposes.
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
          // Disable block delete and throw away character
          // To enable block delete, uncomment following line. Will ignore until EOL.
          // iscomment = true;
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
