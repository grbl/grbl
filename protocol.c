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
#include "report.h"
#include "motion_control.h"

static char line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated.
static uint8_t char_counter; // Last character counter in line variable.
static uint8_t iscomment; // Comment/block delete flag for processor to ignore comment characters.


void protocol_init() 
{
  char_counter = 0; // Reset line input
  iscomment = false;
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
// NOTE: The sys.execute variable flags are set by the serial read subprogram, except where noted,
// but may be set by any process, such as a switch pin change interrupt when pinouts are installed.
void protocol_execute_runtime()
{
  if (sys.execute) { // Enter only if any bit flag is true
    uint8_t rt_exec = sys.execute; // Avoid calling volatile multiple times
    
    // System alarm. Something has gone wrong. Disable everything by entering an infinite
    // loop until system reset/abort.
    if (rt_exec & EXEC_ALARM) {
      if (bit_isfalse(rt_exec,EXEC_RESET)) {  // Ignore loop if reset is already issued
        report_feedback_message(MESSAGE_SYSTEM_ALARM);
        while (bit_isfalse(sys.execute,EXEC_RESET)) { sleep_mode(); }
      }
      bit_false(sys.execute,EXEC_ALARM);
    } 
  
    // System abort. Steppers have already been force stopped.
    if (rt_exec & EXEC_RESET) {
      sys.abort = true; 

    // If the cycle is active before killing the motion, the event will likely caused a loss
    // of position since there is no controlled deceleration(feed hold) to a stop.
    // TODO: Add force home option upon position lost. Need to verify that cycle start isn't
    // set false by anything but the stepper module. Also, need to look at a better place for
    // this. Main.c?
    // if (sys.cycle_start) { protocol_feedback_message(MESSAGE_POSITION_LOST); } 
    
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
      case 'H' : // Perform homing cycle
        if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { mc_go_home(); }
        else { return(STATUS_SETTING_DISABLED); }
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
//    case 'N' : // Start up blocks       
//      if(!read_float(line, &char_counter, &parameter)) { return(STATUS_BAD_NUMBER_FORMAT); }
//      if(line[char_counter++] != '=') { return(STATUS_UNSUPPORTED_STATEMENT); }
//      // Extract startup block, execute, and store.
//      for (char_counter = 0; char_counter < LINE_BUFFER_SIZE-3; char_counter++) {
//        line[char_counter] = line[char_counter+3];
//      }
//      uint8_t status = gc_execute_line(line);
//      if (status) { return(status); }
//      else { settings_store_startup_block(line); }
//      break;
      case 'B' : // Toggle block delete
        if ( line[++char_counter] != 0 ) { return(STATUS_UNSUPPORTED_STATEMENT); }
        sys.switches ^= BITFLAG_BLOCK_DELETE;
        if (bit_istrue(sys.switches,BITFLAG_BLOCK_DELETE)) { report_feedback_message(MESSAGE_SWITCH_ON); }
        else { report_feedback_message(MESSAGE_SWITCH_OFF); }
        break;
      case 'S' : // Toggle single block mode
        if ( line[++char_counter] != 0 ) { return(STATUS_UNSUPPORTED_STATEMENT); }
        sys.switches ^= BITFLAG_SINGLE_BLOCK;
        if (bit_istrue(sys.switches,BITFLAG_SINGLE_BLOCK)) { report_feedback_message(MESSAGE_SWITCH_ON); }
        else { report_feedback_message(MESSAGE_SWITCH_OFF); }
        break;
      case 'O' : // Toggle optional stop
        if ( line[++char_counter] != 0 ) { return(STATUS_UNSUPPORTED_STATEMENT); }
        sys.switches ^= BITFLAG_OPT_STOP;
        if (bit_istrue(sys.switches,BITFLAG_OPT_STOP)) { report_feedback_message(MESSAGE_SWITCH_ON); }
        else { report_feedback_message(MESSAGE_SWITCH_OFF); }
        break;
      default :  // Store global setting
        if(!read_float(line, &char_counter, &parameter)) { return(STATUS_BAD_NUMBER_FORMAT); }
        if(line[char_counter++] != '=') { return(STATUS_UNSUPPORTED_STATEMENT); }
        if(!read_float(line, &char_counter, &value)) { return(STATUS_BAD_NUMBER_FORMAT); }
        if(line[char_counter] != 0) { return(STATUS_UNSUPPORTED_STATEMENT); }
        return(settings_store_global_setting(parameter, value));
    }
    return(STATUS_OK); // If '$' command makes it to here, then everything's ok.

  } else {

    return(gc_execute_line(line));    // Everything else is gcode
    // TODO: Install option to set system alarm upon any error code received back from the
    // the g-code parser. This is a common safety feature on CNCs to help prevent crashes
    // if the g-code doesn't perform as intended.

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
          // Disable block delete and throw away characters. Will ignore until EOL.
          if (bit_istrue(sys.switches,BITFLAG_BLOCK_DELETE)) {
            iscomment = true;
          }
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
