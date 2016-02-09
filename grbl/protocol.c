/*
  protocol.c - controls Grbl execution protocol and procedures
  Part of Grbl
  
  Copyright (c) 2011-2015 Sungeun K. Jeon  
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

#include "grbl.h"

// Define different comment types for pre-parsing.
#define COMMENT_NONE 0
#define COMMENT_TYPE_PARENTHESES 1
#define COMMENT_TYPE_SEMICOLON 2


static char line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated.


// Directs and executes one line of formatted input from protocol_process. While mostly
// incoming streaming g-code blocks, this also directs and executes Grbl internal commands,
// such as settings, initiating the homing cycle, and toggling switch states.
static void protocol_execute_line(char *line) 
{      
  protocol_execute_realtime(); // Runtime command check point.
  if (sys.abort) { return; } // Bail to calling function upon system abort  

  #ifdef REPORT_ECHO_LINE_RECEIVED
    report_echo_line_received(line);
  #endif

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
  GRBL PRIMARY LOOP:
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
    // All systems go! But first check for safety door.
    if (system_check_safety_door_ajar()) {
      bit_true(sys.rt_exec_state, EXEC_SAFETY_DOOR);
      protocol_execute_realtime(); // Enter safety door mode. Should return as IDLE state.
    } else {
      sys.state = STATE_IDLE; // Set system to ready. Clear all state flags.
    } 
    system_execute_startup(line); // Execute startup script.
  }
    
  // ---------------------------------------------------------------------------------  
  // Primary loop! Upon a system abort, this exits back to main() to reset the system. 
  // ---------------------------------------------------------------------------------  
  
  uint8_t comment = COMMENT_NONE;
  uint8_t char_counter = 0;
  uint8_t c;
  for (;;) {

    // Process one line of incoming serial data, as the data becomes available. Performs an
    // initial filtering by removing spaces and comments and capitalizing all letters.
    
    // NOTE: While comment, spaces, and block delete(if supported) handling should technically 
    // be done in the g-code parser, doing it here helps compress the incoming data into Grbl's
    // line buffer, which is limited in size. The g-code standard actually states a line can't
    // exceed 256 characters, but the Arduino Uno does not have the memory space for this.
    // With a better processor, it would be very easy to pull this initial parsing out as a 
    // seperate task to be shared by the g-code parser and Grbl's system commands.
    
    while((c = serial_read()) != SERIAL_NO_DATA) {
      if ((c == '\n') || (c == '\r')) { // End of line reached
        line[char_counter] = 0; // Set string termination character.
        protocol_execute_line(line); // Line is complete. Execute it!
        comment = COMMENT_NONE;
        char_counter = 0;
      } else {
        if (comment != COMMENT_NONE) {
          // Throw away all comment characters
          if (c == ')') {
            // End of comment. Resume line. But, not if semicolon type comment.
            if (comment == COMMENT_TYPE_PARENTHESES) { comment = COMMENT_NONE; }
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
            comment = COMMENT_TYPE_PARENTHESES;
          } else if (c == ';') {
            // NOTE: ';' comment to EOL is a LinuxCNC definition. Not NIST.
            comment = COMMENT_TYPE_SEMICOLON;
            
          // TODO: Install '%' feature 
          // } else if (c == '%') {
            // Program start-end percent sign NOT SUPPORTED.
            // NOTE: This maybe installed to tell Grbl when a program is running vs manual input,
            // where, during a program, the system auto-cycle start will continue to execute 
            // everything until the next '%' sign. This will help fix resuming issues with certain
            // functions that empty the planner buffer to execute its task on-time.

          } else if (char_counter >= (LINE_BUFFER_SIZE-1)) {
            // Detect line buffer overflow. Report error and reset line buffer.
            report_status_message(STATUS_OVERFLOW);
            comment = COMMENT_NONE;
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

    protocol_execute_realtime();  // Runtime command check point.
    if (sys.abort) { return; } // Bail to main() program loop to reset system.
              
  }
  
  return; /* Never reached */
}


// Executes run-time commands, when required. This is called from various check points in the main
// program, primarily where there may be a while loop waiting for a buffer to clear space or any
// point where the execution time from the last check point may be more than a fraction of a second.
// This is a way to execute realtime commands asynchronously (aka multitasking) with grbl's g-code
// parsing and planning functions. This function also serves as an interface for the interrupts to 
// set the system realtime flags, where only the main program handles them, removing the need to
// define more computationally-expensive volatile variables. This also provides a controlled way to 
// execute certain tasks without having two or more instances of the same task, such as the planner
// recalculating the buffer upon a feedhold or override.
// NOTE: The sys.rt_exec_state variable flags are set by any process, step or serial interrupts, pinouts,
// limit switches, or the main program.
void protocol_execute_realtime()
{
  uint8_t rt_exec; // Temp variable to avoid calling volatile multiple times.

  do { // If system is suspended, suspend loop restarts here.
    
  // Check and execute alarms. 
  rt_exec = sys.rt_exec_alarm; // Copy volatile sys.rt_exec_alarm.
  if (rt_exec) { // Enter only if any bit flag is true
    // System alarm. Everything has shutdown by something that has gone severely wrong. Report
    // the source of the error to the user. If critical, Grbl disables by entering an infinite
    // loop until system reset/abort.
    sys.state = STATE_ALARM; // Set system alarm state
    if (rt_exec & EXEC_ALARM_HARD_LIMIT) {
      report_alarm_message(ALARM_HARD_LIMIT_ERROR); 
    } else if (rt_exec & EXEC_ALARM_SOFT_LIMIT) {
      report_alarm_message(ALARM_SOFT_LIMIT_ERROR);
    } else if (rt_exec & EXEC_ALARM_ABORT_CYCLE) {      
      report_alarm_message(ALARM_ABORT_CYCLE);
    } else if (rt_exec & EXEC_ALARM_PROBE_FAIL) {
      report_alarm_message(ALARM_PROBE_FAIL);
    } else if (rt_exec & EXEC_ALARM_HOMING_FAIL) {
      report_alarm_message(ALARM_HOMING_FAIL);
    }
    // Halt everything upon a critical event flag. Currently hard and soft limits flag this.
    if (rt_exec & EXEC_CRITICAL_EVENT) {
      report_feedback_message(MESSAGE_CRITICAL_EVENT);
      bit_false_atomic(sys.rt_exec_state,EXEC_RESET); // Disable any existing reset
      do { 
        // Nothing. Block EVERYTHING until user issues reset or power cycles. Hard limits
        // typically occur while unattended or not paying attention. Gives the user time
        // to do what is needed before resetting, like killing the incoming stream. The 
        // same could be said about soft limits. While the position is not lost, the incoming
        // stream could be still engaged and cause a serious crash if it continues afterwards.
        
        // TODO: Allow status reports during a critical alarm. Still need to think about implications of this.
//         if (sys.rt_exec_state & EXEC_STATUS_REPORT) { 
//           report_realtime_status();
//           bit_false_atomic(sys.rt_exec_state,EXEC_STATUS_REPORT); 
//         }
      } while (bit_isfalse(sys.rt_exec_state,EXEC_RESET));
    }
    bit_false_atomic(sys.rt_exec_alarm,0xFF); // Clear all alarm flags
  }
  
  // Check amd execute realtime commands
  rt_exec = sys.rt_exec_state; // Copy volatile sys.rt_exec_state.
  if (rt_exec) { // Enter only if any bit flag is true
  
    // Execute system abort. 
    if (rt_exec & EXEC_RESET) {
      sys.abort = true;  // Only place this is set true.
      return; // Nothing else to do but exit.
    }
    
    // Execute and serial print status
    if (rt_exec & EXEC_STATUS_REPORT) { 
      report_realtime_status();
      bit_false_atomic(sys.rt_exec_state,EXEC_STATUS_REPORT);
    }
  
    // Execute hold states.
    // NOTE: The math involved to calculate the hold should be low enough for most, if not all, 
    // operational scenarios. Once hold is initiated, the system enters a suspend state to block
    // all main program processes until either reset or resumed.
    if (rt_exec & (EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SAFETY_DOOR)) {
      
      // TODO: CHECK MODE? How to handle this? Likely nothing, since it only works when IDLE and then resets Grbl.
                
      // State check for allowable states for hold methods.
      if ((sys.state == STATE_IDLE) || (sys.state & (STATE_CYCLE | STATE_HOMING | STATE_MOTION_CANCEL | STATE_HOLD | STATE_SAFETY_DOOR))) {

        // If in CYCLE state, all hold states immediately initiate a motion HOLD.
        if (sys.state == STATE_CYCLE) {
          st_update_plan_block_parameters(); // Notify stepper module to recompute for hold deceleration.
          sys.suspend = SUSPEND_ENABLE_HOLD; // Initiate holding cycle with flag.
        }
        // If IDLE, Grbl is not in motion. Simply indicate suspend ready state.
        if (sys.state == STATE_IDLE) { sys.suspend = SUSPEND_ENABLE_READY; }
        
        // Execute and flag a motion cancel with deceleration and return to idle. Used primarily by probing cycle
        // to halt and cancel the remainder of the motion.
        if (rt_exec & EXEC_MOTION_CANCEL) {
          // MOTION_CANCEL only occurs during a CYCLE, but a HOLD and SAFETY_DOOR may been initiated beforehand
          // to hold the CYCLE. If so, only flag that motion cancel is complete.
          if (sys.state == STATE_CYCLE) { sys.state = STATE_MOTION_CANCEL; }
          sys.suspend |= SUSPEND_MOTION_CANCEL; // Indicate motion cancel when resuming. Special motion complete.
        }
    
        // Execute a feed hold with deceleration, only during cycle.
        if (rt_exec & EXEC_FEED_HOLD) {
          // Block SAFETY_DOOR state from prematurely changing back to HOLD.
          if (bit_isfalse(sys.state,STATE_SAFETY_DOOR)) { sys.state = STATE_HOLD; }
        }
  
        // Execute a safety door stop with a feed hold, only during a cycle, and disable spindle/coolant.
        // NOTE: Safety door differs from feed holds by stopping everything no matter state, disables powered
        // devices (spindle/coolant), and blocks resuming until switch is re-engaged. The power-down is 
        // executed here, if IDLE, or when the CYCLE completes via the EXEC_CYCLE_STOP flag.
        if (rt_exec & EXEC_SAFETY_DOOR) {
          report_feedback_message(MESSAGE_SAFETY_DOOR_AJAR); 
          // If already in active, ready-to-resume HOLD, set CYCLE_STOP flag to force de-energize.
          // NOTE: Only temporarily sets the 'rt_exec' variable, not the volatile 'rt_exec_state' variable.
          if (sys.suspend & SUSPEND_ENABLE_READY) { bit_true(rt_exec,EXEC_CYCLE_STOP); }
          sys.suspend |= SUSPEND_ENERGIZE;
          sys.state = STATE_SAFETY_DOOR;
        }
         
      }
      bit_false_atomic(sys.rt_exec_state,(EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SAFETY_DOOR));      
    }
          
    // Execute a cycle start by starting the stepper interrupt to begin executing the blocks in queue.
    if (rt_exec & EXEC_CYCLE_START) {
      // Block if called at same time as the hold commands: feed hold, motion cancel, and safety door.
      // Ensures auto-cycle-start doesn't resume a hold without an explicit user-input.
      if (!(rt_exec & (EXEC_FEED_HOLD | EXEC_MOTION_CANCEL | EXEC_SAFETY_DOOR))) { 
        // Cycle start only when IDLE or when a hold is complete and ready to resume.
        // NOTE: SAFETY_DOOR is implicitly blocked. It reverts to HOLD when the door is closed.
        if ((sys.state == STATE_IDLE) || ((sys.state & (STATE_HOLD | STATE_MOTION_CANCEL)) && (sys.suspend & SUSPEND_ENABLE_READY))) {
          // Re-energize powered components, if disabled by SAFETY_DOOR.
          if (sys.suspend & SUSPEND_ENERGIZE) { 
            // Delayed Tasks: Restart spindle and coolant, delay to power-up, then resume cycle.
            if (gc_state.modal.spindle != SPINDLE_DISABLE) { 
              spindle_set_state(gc_state.modal.spindle, gc_state.spindle_speed); 
              delay_ms(SAFETY_DOOR_SPINDLE_DELAY); // TODO: Blocking function call. Need a non-blocking one eventually.
            }
            if (gc_state.modal.coolant != COOLANT_DISABLE) { 
              coolant_set_state(gc_state.modal.coolant); 
              delay_ms(SAFETY_DOOR_COOLANT_DELAY); // TODO: Blocking function call. Need a non-blocking one eventually.
            }
            // TODO: Install return to pre-park position.
          }
          // Start cycle only if queued motions exist in planner buffer and the motion is not canceled.
          if (plan_get_current_block() && bit_isfalse(sys.suspend,SUSPEND_MOTION_CANCEL)) {
            sys.state = STATE_CYCLE;
            st_prep_buffer(); // Initialize step segment buffer before beginning cycle.
            st_wake_up();
          } else { // Otherwise, do nothing. Set and resume IDLE state.
            sys.state = STATE_IDLE;
          }
          sys.suspend = SUSPEND_DISABLE; // Break suspend state.
        }
      }    
      bit_false_atomic(sys.rt_exec_state,EXEC_CYCLE_START);
    }
    
    // Reinitializes the cycle plan and stepper system after a feed hold for a resume. Called by 
    // realtime command execution in the main program, ensuring that the planner re-plans safely.
    // NOTE: Bresenham algorithm variables are still maintained through both the planner and stepper
    // cycle reinitializations. The stepper path should continue exactly as if nothing has happened.   
    // NOTE: EXEC_CYCLE_STOP is set by the stepper subsystem when a cycle or feed hold completes.
    if (rt_exec & EXEC_CYCLE_STOP) {
      if (sys.state & (STATE_HOLD | STATE_SAFETY_DOOR)) {
        // Hold complete. Set to indicate ready to resume.  Remain in HOLD or DOOR states until user
        // has issued a resume command or reset.
        if (sys.suspend & SUSPEND_ENERGIZE) { // De-energize system if safety door has been opened.
          spindle_stop();
          coolant_stop();
          // TODO: Install parking motion here.
        }
        bit_true(sys.suspend,SUSPEND_ENABLE_READY);
      } else { // Motion is complete. Includes CYCLE, HOMING, and MOTION_CANCEL states.
        sys.suspend = SUSPEND_DISABLE;
        sys.state = STATE_IDLE;
      }
      bit_false_atomic(sys.rt_exec_state,EXEC_CYCLE_STOP);
    }
    
  }

  // Overrides flag byte (sys.override) and execution should be installed here, since they 
  // are realtime and require a direct and controlled interface to the main stepper program.

  // Reload step segment buffer
  if (sys.state & (STATE_CYCLE | STATE_HOLD | STATE_MOTION_CANCEL | STATE_SAFETY_DOOR | STATE_HOMING)) { st_prep_buffer(); }  
  
  // If safety door was opened, actively check when safety door is closed and ready to resume.
  // NOTE: This unlocks the SAFETY_DOOR state to a HOLD state, such that CYCLE_START can activate a resume.
  if (sys.state == STATE_SAFETY_DOOR) { 
    if (bit_istrue(sys.suspend,SUSPEND_ENABLE_READY)) { 
      if (!(system_check_safety_door_ajar())) {
        sys.state = STATE_HOLD; // Update to HOLD state to indicate door is closed and ready to resume.
      }
    }
  }

  } while(sys.suspend); // Check for system suspend state before exiting.
  
}  


// Block until all buffered steps are executed or in a cycle state. Works with feed hold
// during a synchronize call, if it should happen. Also, waits for clean cycle end.
void protocol_buffer_synchronize()
{
  // If system is queued, ensure cycle resumes if the auto start flag is present.
  protocol_auto_cycle_start();
  do {
    protocol_execute_realtime();   // Check and execute run-time commands
    if (sys.abort) { return; } // Check for system abort
  } while (plan_get_current_block() || (sys.state == STATE_CYCLE));
}


// Auto-cycle start has two purposes: 1. Resumes a plan_synchronize() call from a function that
// requires the planner buffer to empty (spindle enable, dwell, etc.) 2. As a user setting that 
// automatically begins the cycle when a user enters a valid motion command manually. This is 
// intended as a beginners feature to help new users to understand g-code. It can be disabled
// as a beginner tool, but (1.) still operates. If disabled, the operation of cycle start is
// manually issuing a cycle start command whenever the user is ready and there is a valid motion 
// command in the planner queue.
// NOTE: This function is called from the main loop, buffer sync, and mc_line() only and executes 
// when one of these conditions exist respectively: There are no more blocks sent (i.e. streaming 
// is finished, single commands), a command that needs to wait for the motions in the buffer to 
// execute calls a buffer sync, or the planner buffer is full and ready to go.
void protocol_auto_cycle_start() { bit_true_atomic(sys.rt_exec_state, EXEC_CYCLE_START); } 
