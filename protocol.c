/*
  protocol.c - controls Grbl execution procedures
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
#include "stepper.h"
#include "motion_control.h"
#include "report.h"


static char line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated.


// Executes run-time commands, when required. This is called from various check points in the main
// program, primarily where there may be a while loop waiting for a buffer to clear space or any
// point where the execution time from the last check point may be more than a fraction of a second.
// This is a way to execute runtime commands asynchronously (aka multitasking) with grbl's g-code
// parsing and planning functions. This function also serves as an interface for the interrupts to 
// set the system runtime flags, where only the main program handles them, removing the need to
// define more computationally-expensive volatile variables. This also provides a controlled way to 
// execute certain tasks without having two or more instances of the same task, such as the planner
// recalculating the buffer upon a feedhold or override.
// NOTE: The sys.execute variable flags are set by any process, step or serial interrupts, pinouts,
// limit switches, or the main program.
void protocol_execute_runtime()
{
  // Reload step segment buffer
  st_prep_buffer();
  
  if (sys.execute) { // Enter only if any bit flag is true
    uint8_t rt_exec = sys.execute; // Avoid calling volatile multiple times
    
    // System alarm. Everything has shutdown by something that has gone severely wrong. Report
    // the source of the error to the user. If critical, Grbl disables by entering an infinite
    // loop until system reset/abort.
    if (rt_exec & (EXEC_ALARM | EXEC_CRIT_EVENT)) {      
      sys.state = STATE_ALARM; // Set system alarm state

      // Critical event. Only hard/soft limit errors currently qualify.
      if (rt_exec & EXEC_CRIT_EVENT) {
        report_alarm_message(ALARM_LIMIT_ERROR); 
        report_feedback_message(MESSAGE_CRITICAL_EVENT);
        bit_false(sys.execute,EXEC_RESET); // Disable any existing reset
        do { 
          // Nothing. Block EVERYTHING until user issues reset or power cycles. Hard limits
          // typically occur while unattended or not paying attention. Gives the user time
          // to do what is needed before resetting, like killing the incoming stream. The 
          // same could be said about soft limits. While the position is not lost, the incoming
          // stream could be still engaged and cause a serious crash if it continues afterwards.
        } while (bit_isfalse(sys.execute,EXEC_RESET));

      // Standard alarm event. Only abort during motion qualifies.
      } else {
        // Runtime abort command issued during a cycle, feed hold, or homing cycle. Message the
        // user that position may have been lost and set alarm state to enable the alarm lockout
        // to indicate the possible severity of the problem.
        report_alarm_message(ALARM_ABORT_CYCLE);
      }
      bit_false(sys.execute,(EXEC_ALARM | EXEC_CRIT_EVENT));
    } 
  
    // Execute system abort. 
    if (rt_exec & EXEC_RESET) {
      sys.abort = true;  // Only place this is set true.
      return; // Nothing else to do but exit.
    }
    
    // Execute and serial print status
    if (rt_exec & EXEC_STATUS_REPORT) { 
      report_realtime_status();
      bit_false(sys.execute,EXEC_STATUS_REPORT);
    }
    
    // Initiate stepper feed hold
    if (rt_exec & EXEC_FEED_HOLD) {
      // !!! During a cycle, the segment buffer has just been reloaded and full. So the math involved
      // with the feed hold should be fine for most, if not all, operational scenarios.
      st_feed_hold(); // Initiate feed hold.
      bit_false(sys.execute,EXEC_FEED_HOLD);
    }
    
    // Reinitializes the stepper module running state and, if a feed hold, re-plans the buffer.
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
  
  // Overrides flag byte (sys.override) and execution should be installed here, since they 
  // are runtime and require a direct and controlled interface to the main stepper program.
}  


// Directs and executes one line of formatted input from protocol_process. While mostly
// incoming streaming g-code blocks, this also directs and executes Grbl internal commands,
// such as settings, initiating the homing cycle, and toggling switch states.
// TODO: Eventually re-organize this function to more cleanly organize order of operations,
// which will hopefully reduce some of the current spaghetti logic and dynamic memory usage. 
static void protocol_execute_line(char *line) 
{      
  protocol_execute_runtime(); // Runtime command check point.
  if (sys.abort) { return; } // Bail to calling function upon system abort  

  uint8_t status;
  if (line[0] == 0) {
    // Empty or comment line. Send status message for syncing purposes.
    status = STATUS_OK;

  } else if (line[0] == '$') {
    // Grbl '$' system command
    status = system_execute_line(line);
    
  } else {
    // Everything else is gcode. Send to g-code parser!
    // TODO: Separate the parsing from the g-code execution. Need to re-write the parser
    // completely to do this. First parse the line completely, checking for modal group 
    // errors and storing all of the g-code words. Then, send the stored g-code words to
    // a separate g-code executor. This will be more in-line with actual g-code protocol.
    status = gc_execute_line(line);
  
  }
  
  report_status_message(status);
}


void protocol_process()
{
  // ------------------------------------------------------------
  // Complete initialization procedures upon a power-up or reset.
  // ------------------------------------------------------------
  
  // Print welcome message   
  report_init_message();

  // Check for and report alarm state after a reset, error, or an initial power up.
  if (sys.state == STATE_ALARM) {
    report_feedback_message(MESSAGE_ALARM_LOCK); 
  } else {
    // All systems go!
    sys.state = STATE_IDLE; // Set system to ready. Clear all state flags.
    system_execute_startup(line); // Execute startup script.
  }
    
  // ------------------------------------------------------------------------------  
  // Main loop! Upon a system abort, this exits back to main() to reset the system. 
  // ------------------------------------------------------------------------------  
  
  uint8_t iscomment = false;
  uint8_t char_counter = 0;
  uint8_t c;
  for (;;) {

    // Process one line of incoming serial data, as the data becomes available. Performs an
    // initial filtering by removing spaces and comments and capitalizing all letters.
    while((c = serial_read()) != SERIAL_NO_DATA) {
      if ((c == '\n') || (c == '\r')) { // End of line reached
        line[char_counter] = 0; // Set string termination character.
        protocol_execute_line(line); // Line is complete. Execute it!
        iscomment = false;
        char_counter = 0;
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
            // Block delete not supported. Ignore character.
          } else if (c == '(') {
            // Enable comments flag and ignore all characters until ')' or EOL.
            iscomment = true;
          } else if (char_counter >= LINE_BUFFER_SIZE-1) {
            // Detect line buffer overflow. Report error and reset line buffer.
            report_status_message(STATUS_OVERFLOW);
            iscomment = false;
            char_counter = 0;
          } else if (c >= 'a' && c <= 'z') { // Upcase lowercase
            line[char_counter++] = c-'a'+'A';
          } else {
            line[char_counter++] = c;
          }
        }
      }
    }
    
    protocol_execute_runtime();  // Runtime command check point.
    if (sys.abort) { return; } // Bail to main() program loop to reset system.
          
    // If there are no more characters in the serial read buffer to be processed and executed,
    // this indicates that g-code streaming has either filled the planner buffer or has 
    // completed. In either case, auto-cycle start, if enabled, any queued moves.
    mc_auto_cycle_start();
    
  }
  
  return; /* Never reached */
}
