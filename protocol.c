/*
  protocol.c - controls Grbl execution protocol and procedures
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
#include "motion_control.h"
#include "report.h"


static char line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated.


// Directs and executes one line of formatted input from protocol_process. While mostly
// incoming streaming g-code blocks, this also directs and executes Grbl internal commands,
// such as settings, initiating the homing cycle, and toggling switch states.
static void protocol_execute_line(char *line) 
{      
  protocol_execute_runtime(); // Runtime command check point.
  if (sys.abort) { return; } // Bail to calling function upon system abort  

  if (line[0] == 0) {
    // Empty or comment line. Send status message for syncing purposes.
    report_status_message(STATUS_OK);

  } else if (line[0] == '$') {
    // Grbl '$' system command
    report_status_message(system_execute_line(line));
    
  } else if (sys.state == STATE_ALARM) {
    // Everything else is gcode. Block if in alarm mode.
    report_status_message(STATUS_ALARM_LOCK);

  } else {
    // Parse and execute g-code block!
    report_status_message(gc_execute_line(line));
  }
}


/* 
  GRBL MAIN LOOP:
*/
void protocol_main_loop()
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
            // Block delete NOT SUPPORTED. Ignore character.
            // NOTE: If supported, would simply need to check the system if block delete is enabled.
          } else if (c == '(') {
            // Enable comments flag and ignore all characters until ')' or EOL.
            // NOTE: This doesn't follow the NIST definition exactly, but is good enough for now.
            // In the future, we could simply remove the items within the comments, but retain the
            // comment control characters, so that the g-code parser can error-check it.
            iscomment = true;
          // } else if (c == ';') {
            // Comment character to EOL NOT SUPPORTED. LinuxCNC definition. Not NIST.
            
          // TODO: Install '%' feature 
          // } else if (c == '%') {
            // Program start-end percent sign NOT SUPPORTED.
            // NOTE: This maybe installed to tell Grbl when a program is running vs manual input,
            // where, during a program, the system auto-cycle start will continue to execute 
            // everything until the next '%' sign. This will help fix resuming issues with certain
            // functions that empty the planner buffer to execute its task on-time.

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
    
    // If there are no more characters in the serial read buffer to be processed and executed,
    // this indicates that g-code streaming has either filled the planner buffer or has 
    // completed. In either case, auto-cycle start, if enabled, any queued moves.
    protocol_auto_cycle_start();

    protocol_execute_runtime();  // Runtime command check point.
    if (sys.abort) { return; } // Bail to main() program loop to reset system.
              
  }
  
  return; /* Never reached */
}


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
  uint8_t rt_exec = sys.execute; // Copy to avoid calling volatile multiple times
  if (rt_exec) { // Enter only if any bit flag is true
    
    // System alarm. Everything has shutdown by something that has gone severely wrong. Report
    // the source of the error to the user. If critical, Grbl disables by entering an infinite
    // loop until system reset/abort.
    if (rt_exec & (EXEC_ALARM | EXEC_CRIT_EVENT)) {      
      sys.state = STATE_ALARM; // Set system alarm state

      // Critical events. Hard/soft limit events identified by both critical event and alarm exec
      // flags. Probe fail is identified by the critical event exec flag only.
      if (rt_exec & EXEC_CRIT_EVENT) {
        if (rt_exec & EXEC_ALARM) { report_alarm_message(ALARM_LIMIT_ERROR); }
        else { report_alarm_message(ALARM_PROBE_FAIL); }
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
    
    // Execute a feed hold with deceleration, only during cycle.
    if (rt_exec & EXEC_FEED_HOLD) {
      // !!! During a cycle, the segment buffer has just been reloaded and full. So the math involved
      // with the feed hold should be fine for most, if not all, operational scenarios.
      if (sys.state == STATE_CYCLE) {
        sys.state = STATE_HOLD;
        st_update_plan_block_parameters();
        st_prep_buffer();
        sys.auto_start = false; // Disable planner auto start upon feed hold.
      }
      bit_false(sys.execute,EXEC_FEED_HOLD);
    }
        
    // Execute a cycle start by starting the stepper interrupt begin executing the blocks in queue.
    if (rt_exec & EXEC_CYCLE_START) { 
      if (sys.state == STATE_QUEUED) {
        sys.state = STATE_CYCLE;
        st_prep_buffer(); // Initialize step segment buffer before beginning cycle.
        st_wake_up();
        if (bit_istrue(settings.flags,BITFLAG_AUTO_START)) {
          sys.auto_start = true; // Re-enable auto start after feed hold.
        } else {
          sys.auto_start = false; // Reset auto start per settings.
        }
      }    
      bit_false(sys.execute,EXEC_CYCLE_START);
    }
    
    // Reinitializes the cycle plan and stepper system after a feed hold for a resume. Called by 
    // runtime command execution in the main program, ensuring that the planner re-plans safely.
    // NOTE: Bresenham algorithm variables are still maintained through both the planner and stepper
    // cycle reinitializations. The stepper path should continue exactly as if nothing has happened.   
    // NOTE: EXEC_CYCLE_STOP is set by the stepper subsystem when a cycle or feed hold completes.
    if (rt_exec & EXEC_CYCLE_STOP) {
      if ( plan_get_current_block() ) { sys.state = STATE_QUEUED; }
      else { sys.state = STATE_IDLE; }
      bit_false(sys.execute,EXEC_CYCLE_STOP);
    }

  }
  
  // Overrides flag byte (sys.override) and execution should be installed here, since they 
  // are runtime and require a direct and controlled interface to the main stepper program.

  // Reload step segment buffer
  if (sys.state & (STATE_CYCLE | STATE_HOLD | STATE_HOMING)) { st_prep_buffer(); }  
  
}  


// Block until all buffered steps are executed or in a cycle state. Works with feed hold
// during a synchronize call, if it should happen. Also, waits for clean cycle end.
void protocol_buffer_synchronize()
{
  // Check and set auto start to resume cycle after synchronize and caller completes.
  if (sys.state == STATE_CYCLE) { sys.auto_start = true; }
  while (plan_get_current_block() || (sys.state == STATE_CYCLE)) { 
    protocol_execute_runtime();   // Check and execute run-time commands
    if (sys.abort) { return; } // Check for system abort
  }    
}


// Auto-cycle start has two purposes: 1. Resumes a plan_synchronize() call from a function that
// requires the planner buffer to empty (spindle enable, dwell, etc.) 2. As a user setting that 
// automatically begins the cycle when a user enters a valid motion command manually. This is 
// intended as a beginners feature to help new users to understand g-code. It can be disabled
// as a beginner tool, but (1.) still operates. If disabled, the operation of cycle start is
// manually issuing a cycle start command whenever the user is ready and there is a valid motion 
// command in the planner queue.
// NOTE: This function is called from the main loop and mc_line() only and executes when one of
// two conditions exist respectively: There are no more blocks sent (i.e. streaming is finished, 
// single commands), or the planner buffer is full and ready to go.
void protocol_auto_cycle_start() { if (sys.auto_start) { sys.execute |= EXEC_CYCLE_START; } } 
