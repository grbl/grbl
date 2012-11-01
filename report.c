/*
  report.c - reporting and messaging methods
  Part of Grbl

  Copyright (c) 2012 Sungeun K. Jeon  

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

/* 
  This file functions as the primary feedback interface for Grbl. Any outgoing data, such 
  as the protocol status messages, feedback messages, and status reports, are stored here.
  For the most part, these functions primarily are called from protocol.c methods. If a 
  different style feedback is desired (i.e. JSON), then a user can change these following 
  methods to accomodate their needs.
*/

#include <avr/io.h>
#include "print.h"
#include "settings.h"
#include <math.h>
#include "nuts_bolts.h"
#include <avr/pgmspace.h>
#include "report.h"
#include "protocol.h"


// Handles the primary confirmation protocol response for streaming interfaces and human-feedback.
// For every incoming line, this method responds with an 'ok' for a successful command or an 
// 'error:'  to indicate some error event with the line or some critical system error during 
// operation. Errors events can originate from the g-code parser, settings module, or asynchronously
// from a critical error, such as a triggered hard limit. Interface should always monitor for these
// responses.
// NOTE: In silent mode, all error codes are greater than zero.
// TODO: Install silent mode to return only numeric values, primarily for GUIs.
void report_status_message(uint8_t status_code) 
{
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
      case STATUS_SETTING_VALUE_NEG:
      printPgmString(PSTR("Set value must be > 0.0")); break;
      case STATUS_SETTING_STEP_PULSE_MIN:
      printPgmString(PSTR("Step pulse must be >= 3 microseconds")); break;
      case STATUS_SETTING_READ_FAIL:
      printPgmString(PSTR("Failed to read EEPROM settings. Using defaults")); break;
    }
    printPgmString(PSTR("\r\n"));
  }
}


// Prints feedback messages. This serves as a centralized method to provide additional
// user feedback for things that are not of the status message response protocol. These 
// are messages such as setup warnings and how to exit alarms.
// NOTE: For interfaces, messages are always placed within parentheses. And if silent mode
// is installed, the message codes are less than zero.
// TODO: Install silence feedback messages option in settings
void report_feedback_message(int8_t message_code)
{
  printPgmString(PSTR("("));
  switch(message_code) {
    case MESSAGE_SYSTEM_ALARM:
    printPgmString(PSTR("ALARM: Check and reset Grbl")); break;
    case MESSAGE_POSITION_LOST:
    printPgmString(PSTR("warning: Position may be lost")); break;
    case MESSAGE_HOMING_ENABLE:
    printPgmString(PSTR("'$H' to home. Ensure all limit switches are installed")); break;
  }
  printPgmString(PSTR(")\r\n"));
}

// Welcome message
void report_init_message()
{
  // Print grbl initialization message
  printPgmString(PSTR("\r\nGrbl " GRBL_VERSION));
  printPgmString(PSTR("\r\n'$' for help and to view settings\r\n"));
}


void report_grbl_help() {
  printPgmString(PSTR("$0 = ")); printFloat(settings.steps_per_mm[X_AXIS]);
  printPgmString(PSTR(" (x axis, steps/mm)\r\n$1 = ")); printFloat(settings.steps_per_mm[Y_AXIS]);
  printPgmString(PSTR(" (y axis, steps/mm)\r\n$2 = ")); printFloat(settings.steps_per_mm[Z_AXIS]);
  printPgmString(PSTR(" (z axis, steps/mm)\r\n$3 = ")); printInteger(settings.pulse_microseconds);
  printPgmString(PSTR(" (step pulse, usec)\r\n$4 = ")); printFloat(settings.default_feed_rate);
  printPgmString(PSTR(" (default feed rate, mm/min)\r\n$5 = ")); printFloat(settings.default_seek_rate);
  printPgmString(PSTR(" (default seek rate, mm/min)\r\n$6 = ")); printFloat(settings.mm_per_arc_segment);
  printPgmString(PSTR(" (arc resolution, mm/segment)\r\n$7 = ")); printInteger(settings.invert_mask); 
  printPgmString(PSTR(" (step port invert mask, int:binary = ")); print_uint8_base2(settings.invert_mask);  
  printPgmString(PSTR(")\r\n$8 = ")); printFloat(settings.acceleration/(60*60)); // Convert from mm/min^2 for human readability
  printPgmString(PSTR(" (acceleration, mm/sec^2)\r\n$9 = ")); printFloat(settings.junction_deviation);
  printPgmString(PSTR(" (cornering junction deviation, mm)\r\n$10 = ")); printInteger(bit_istrue(settings.flags,BITFLAG_REPORT_INCHES));
  printPgmString(PSTR(" (status report inches, bool)\r\n$11 = ")); printInteger(bit_istrue(settings.flags,BITFLAG_AUTO_START));
  printPgmString(PSTR(" (auto start enable, bool)\r\n$12 = ")); printInteger(bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE));
  printPgmString(PSTR(" (invert stepper enable, bool)\r\n$13 = ")); printInteger(bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE));
  printPgmString(PSTR(" (hard limit enable, bool)\r\n$14 = ")); printInteger(bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE));
  printPgmString(PSTR(" (homing enable, bool)\r\n$15 = ")); printInteger(settings.homing_dir_mask);
  printPgmString(PSTR(" (homing direction mask, int:binary = ")); print_uint8_base2(settings.homing_dir_mask);  
  printPgmString(PSTR(")\r\n$16 = ")); printFloat(settings.homing_feed_rate);
  printPgmString(PSTR(" (homing feed rate, mm/min)\r\n$17 = ")); printFloat(settings.homing_seek_rate);
  printPgmString(PSTR(" (homing seek rate, mm/min)\r\n$18 = ")); printInteger(settings.homing_debounce_delay);
  printPgmString(PSTR(" (homing debounce delay, msec)\r\n$19 = ")); printFloat(settings.homing_pulloff);
  printPgmString(PSTR(" (homing pull-off travel, mm)\r\n$20 = ")); printInteger(settings.stepper_idle_lock_time);
  printPgmString(PSTR(" (stepper idle lock time, msec)\r\n$21 = ")); printInteger(settings.decimal_places);
  printPgmString(PSTR(" (decimal places, int)\r\n$22 = ")); printInteger(settings.n_arc_correction);
//   char st_line[LINE_BUFFER_SIZE];
//   printPgmString(PSTR("\r\n\r\n Startup\r\n$100 = ")); printString(settings_read_startup(st_line,0));
//   printPgmString(PSTR("\r\n$101 = ")); printString(settings_read_startup(st_line,1));
  
//   char buf[4];
//   settings_startup_string((char *)buf);
//   printPgmString(PSTR("\r\n Startup: ")); printString(buf);
  
  printPgmString(PSTR("\r\n'$x=value' to store setting"));
//  printPgmString(PSTR("\r\n'$Sx' to toggle switch\r\n"));
}



// void report_gcode_status()
// {
//   // Print gcode parser state
// }
// 
// void report_gcode_parameters()
// {
//   // Print gcode work coordinate offsets?
// }


void report_realtime_status()
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
     print_position[i] -= (sys.coord_system[i]+sys.coord_offset[i])*INCH_PER_MM;
   } else {
     print_position[i] -= sys.coord_system[i]+sys.coord_offset[i];
   }
   printFloat(print_position[i]);
   if (i < 2) { printPgmString(PSTR(",")); }
   else { printPgmString(PSTR("]")); }
 }
   
 printPgmString(PSTR("\r\n"));
}
