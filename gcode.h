/*
  gcode.c - rs274/ngc parser.
  Part of Grbl

  Copyright (c) 2009 Simen Svale Skogsrud

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


#ifndef gcode_h
#define gcode_h
#include <avr/io.h>

#define GCSTATUS_OK                      0
#define GCSTATUS_BAD_NUMBER_FORMAT       1
#define GCSTATUS_EXPECTED_COMMAND_LETTER 2
#define GCSTATUS_UNSUPPORTED_STATEMENT   3
#define GCSTATUS_MOTION_CONTROL_ERROR    4
#define GCSTATUS_FLOATING_POINT_ERROR    5
#define GCSTATUS_UNSUPPORTED_LETTER 	 6
#define GCSTATUS_BUFFER_FULL			 7

#define MOTION_MODE_SEEK    0  // G0
#define MOTION_MODE_LINEAR  1  // G1
#define MOTION_MODE_CW_ARC  2  // G2
#define MOTION_MODE_CCW_ARC 3  // G3
#define MOTION_MODE_CANCEL  4  // G80



struct ParserState {
  uint8_t status_code;            // Current GCSTATUS_* of the interpreter
  uint8_t motion_mode;            // {G0, G1, G2, G3, G80}
  uint8_t inverse_feed_rate_mode; // G93, G94
  uint8_t inches_mode;            // 0 = millimeter mode, 1 = inches mode {G20, G21}
  uint8_t absolute_mode;          // 0 = relative motion, 1 = absolute motion {G90, G91}
  uint8_t program_flow;           // Follows G-code specs for no apparent reason. Currently unsupported
  int spindle_direction;          // +1 == forward, -1 == reverse, 0 == stopped (handled in the spindle_control module)
  double feed_rate, seek_rate;               // Millimeters/second
  double position[3];             // The current location of the tool at this point in the code (will generally diverge from current reality)
  uint8_t tool;                   // The currently loaded tool (currently unsupported)
  int16_t spindle_speed;          // RPM/100 (currently unsupported in spindle_control)
  uint8_t plane_axis_0,           // The axes of the selected plane
          plane_axis_1,
          plane_axis_2;

};
extern struct ParserState gc;

// Initialize the parser
void gc_init();

// Execute one block of rs275/ngc/g-code
uint8_t gc_execute_line(char *line);

void set_gcPosition();

#endif
