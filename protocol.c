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
#include <avr/interrupt.h>
#include "protocol.h"
#include "gcode.h"
#include "serial.h"
#include "print.h"
#include "settings.h"
#include "config.h"
#include "nuts_bolts.h"
#include "stepper.h"
#include "report.h"
#include "motion_control.h"

static char line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated.
static uint8_t char_counter; // Last character counter in line variable.
static uint8_t iscomment; // Comment/block delete flag for processor to ignore comment characters.


void protocol_init() 
{
  char_counter = 0; // Reset line input
  iscomment = false;
  report_init_message(); // Welcome message   
  
  PINOUT_DDR &= ~(PINOUT_MASK); // Set as input pins
  PINOUT_PORT |= PINOUT_MASK; // Enable internal pull-up resistors. Normal high operation.
  PINOUT_PCMSK |= PINOUT_MASK;   // Enable specific pins of the Pin Change Interrupt
  PCICR |= (1 << PINOUT_INT);   // Enable Pin Change Interrupt
}

// Executes user startup script, if stored.
void protocol_execute_startup() 
{
  uint8_t n;
  for (n=0; n < N_STARTUP_LINE; n++) {
    if (!(settings_read_startup_line(n, line))) {
      report_status_message(STATUS_SETTING_READ_FAIL);
    } else {
      if (line[0] != 0) {
        printString(line); // Echo startup line to indicate execution.
        report_status_message(gc_execute_line(line));
      }
    } 
  }  
}

// Pin change interrupt for pin-out commands, i.e. cycle start, feed hold, and reset. Sets
// only the runtime command execute variable to have the main program execute these when 
// its ready. This works exactly like the character-based runtime commands when picked off
// directly from the incoming serial data stream.
ISR(PINOUT_INT_vect) 
{
  // Enter only if any pinout pin is actively low.
  if ((PINOUT_PIN & PINOUT_MASK) ^ PINOUT_MASK) { 
    if (bit_isfalse(PINOUT_PIN,bit(PIN_RESET))) {
      mc_reset();
    } else if (bit_isfalse(PINOUT_PIN,bit(PIN_FEED_HOLD))) {
      sys.execute |= EXEC_FEED_HOLD; 
    } else if (bit_isfalse(PINOUT_PIN,bit(PIN_CYCLE_START))) {
      sys.execute |= EXEC_CYCLE_START;
    }
  }
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
  if (sys.execute) { // Enter only if any bit flag is true
    uint8_t rt_exec = sys.execute; // Avoid calling volatile multiple times
    
    // System alarm. Everything has shutdown by something that has gone severely wrong. Report
    // the source of the error to the user. If critical, Grbl disables by entering an infinite
    // loop until system reset/abort.
    if (rt_exec & (EXEC_ALARM | EXEC_CRIT_EVENT)) {      
      sys.state = STATE_ALARM; // Set system alarm state

      // Critical event. Only hard limit qualifies. Update this as new critical events surface.
      if (rt_exec & EXEC_CRIT_EVENT) {
        report_alarm_message(ALARM_HARD_LIMIT); 
        report_feedback_message(MESSAGE_CRITICAL_EVENT);
        bit_false(sys.execute,EXEC_RESET); // Disable any existing reset
        do { 
          // Nothing. Block EVERYTHING until user issues reset or power cycles. Hard limits
          // typically occur while unattended or not paying attention. Gives the user time
          // to do what is needed before resetting, like killing the incoming stream.
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
// incoming streaming g-code blocks, this also executes Grbl internal commands, such as 
// settings, initiating the homing cycle, and toggling switch states. This differs from
// the runtime command module by being susceptible to when Grbl is ready to execute the 
// next line during a cycle, so for switches like block delete, the switch only effects
// the lines that are processed afterward, not necessarily real-time during a cycle, 
// since there are motions already stored in the buffer. However, this 'lag' should not
// be an issue, since these commands are not typically used during a cycle.
uint8_t protocol_execute_line(char *line) 
{   
  // Grbl internal command and parameter lines are of the form '$4=374.3' or '$' for help  
  if(line[0] == '$') {
    
    uint8_t char_counter = 1; 
    uint8_t helper_var = 0; // Helper variable
    float parameter, value;
    switch( line[char_counter] ) {
      case 0 : report_grbl_help(); break;
      case '$' : // Prints Grbl settings
        if ( line[++char_counter] != 0 ) { return(STATUS_UNSUPPORTED_STATEMENT); }
        else { report_grbl_settings(); }
        break;
      case '#' : // Print gcode parameters
        if ( line[++char_counter] != 0 ) { return(STATUS_UNSUPPORTED_STATEMENT); }
        else { report_gcode_parameters(); }
        break;
      case 'G' : // Prints gcode parser state
        if ( line[++char_counter] != 0 ) { return(STATUS_UNSUPPORTED_STATEMENT); }
        else { report_gcode_modes(); }
        break;
      case 'C' : // Set check g-code mode
        if ( line[++char_counter] != 0 ) { return(STATUS_UNSUPPORTED_STATEMENT); }
        // Perform reset when toggling off. Check g-code mode should only work if Grbl
        // is idle and ready, regardless of alarm locks. This is mainly to keep things
        // simple and consistent.
        if ( sys.state == STATE_CHECK_MODE ) { 
          mc_reset(); 
          report_feedback_message(MESSAGE_DISABLED);
        } else {
          if (sys.state) { return(STATUS_IDLE_ERROR); }
          sys.state = STATE_CHECK_MODE;
          report_feedback_message(MESSAGE_ENABLED);
        }
        break; 
      case 'X' : // Disable alarm lock
        if ( line[++char_counter] != 0 ) { return(STATUS_UNSUPPORTED_STATEMENT); }
        if (sys.state == STATE_ALARM) { 
          report_feedback_message(MESSAGE_ALARM_UNLOCK);
          sys.state = STATE_IDLE;
          // Don't run startup script. Prevents stored moves in startup from causing accidents.
        }
        break;               
      case 'H' : // Perform homing cycle
        if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { 
          // Only perform homing if Grbl is idle or lost.
          if ( sys.state==STATE_IDLE || sys.state==STATE_ALARM ) { 
            mc_go_home(); 
            if (!sys.abort) { protocol_execute_startup(); } // Execute startup scripts after successful homing.
          } else { return(STATUS_IDLE_ERROR); }
        } else { return(STATUS_SETTING_DISABLED); }
        break;
//    case 'J' : break;  // Jogging methods
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
      case 'N' : // Startup lines. 
        if ( line[++char_counter] == 0 ) { // Print startup lines
          for (helper_var=0; helper_var < N_STARTUP_LINE; helper_var++) {
            if (!(settings_read_startup_line(helper_var, line))) {
              report_status_message(STATUS_SETTING_READ_FAIL);
            } else {
              report_startup_line(helper_var,line);
            }
          }
          break;
        } else { // Store startup line
          helper_var = true;  // Set helper_var to flag storing method. 
          // No break. Continues into default: to read remaining command characters.
        }
      default :  // Storing setting methods
        if(!read_float(line, &char_counter, &parameter)) { return(STATUS_BAD_NUMBER_FORMAT); }
        if(line[char_counter++] != '=') { return(STATUS_UNSUPPORTED_STATEMENT); }
        if (helper_var) { // Store startup line
          // Prepare sending gcode block to gcode parser by shifting all characters
          helper_var = char_counter; // Set helper variable as counter to start of gcode block
          do {
            line[char_counter-helper_var] = line[char_counter];
          } while (line[char_counter++] != 0);
          // Execute gcode block to ensure block is valid.
          helper_var = gc_execute_line(line); // Set helper_var to returned status code.
          if (helper_var) { return(helper_var); }
          else { 
            helper_var = trunc(parameter); // Set helper_var to int value of parameter
            settings_store_startup_line(helper_var,line);
          }
        } else { // Store global setting.
          if(!read_float(line, &char_counter, &value)) { return(STATUS_BAD_NUMBER_FORMAT); }
          if(line[char_counter] != 0) { return(STATUS_UNSUPPORTED_STATEMENT); }
          return(settings_store_global_setting(parameter, value));
        }
    }
    return(STATUS_OK); // If '$' command makes it to here, then everything's ok.

  } else {
    return(gc_execute_line(line));    // Everything else is gcode
  }
}


// Process and report status one line of incoming serial data. Performs an initial filtering
// by removing spaces and comments and capitalizing all letters.
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
        report_status_message(protocol_execute_line(line));
      } else { 
        // Empty or comment line. Skip block.
        report_status_message(STATUS_OK); // Send status message for syncing purposes.
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
          // Block delete not supported. Ignore character.
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
