/*
  gcode.h - rs274/ngc parser.
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

#ifndef gcode_h
#define gcode_h

#include "system.h"

// Define modal group internal numbers for checking multiple command violations and tracking the 
// type of command that is called in the block. A modal group is a group of g-code commands that are
// mutually exclusive, or cannot exist on the same line, because they each toggle a state or execute
// a unique motion. These are defined in the NIST RS274-NGC v3 g-code standard, available online, 
// and are similar/identical to other g-code interpreters by manufacturers (Haas,Fanuc,Mazak,etc).
#define MODAL_GROUP_NONE 0
#define MODAL_GROUP_0 1 // [G4,G10,G28,G30,G53,G92,G92.1] Non-modal
#define MODAL_GROUP_1 2 // [G0,G1,G2,G3,G80] Motion
#define MODAL_GROUP_2 3 // [G17,G18,G19] Plane selection
#define MODAL_GROUP_3 4 // [G90,G91] Distance mode
#define MODAL_GROUP_4 5 // [M0,M1,M2,M30] Stopping
#define MODAL_GROUP_5 6 // [G93,G94] Feed rate mode
#define MODAL_GROUP_6 7 // [G20,G21] Units
#define MODAL_GROUP_7 8 // [M3,M4,M5] Spindle turning
#define MODAL_GROUP_8 9 // [M7,M8,M9] Coolant control
#define MODAL_GROUP_12 10 // [G54,G55,G56,G57,G58,G59] Coordinate system selection

// Define command actions for within execution-type modal groups (motion, stopping, non-modal). Used
// internally by the parser to know which command to execute.
#define MOTION_MODE_SEEK 0 // G0 
#define MOTION_MODE_LINEAR 1 // G1
#define MOTION_MODE_CW_ARC 2  // G2
#define MOTION_MODE_CCW_ARC 3  // G3
#define MOTION_MODE_CANCEL 4 // G80

#define PROGRAM_FLOW_RUNNING 0
#define PROGRAM_FLOW_PAUSED 1 // M0, M1
#define PROGRAM_FLOW_COMPLETED 2 // M2, M30

#define NON_MODAL_NONE 0
#define NON_MODAL_DWELL 1 // G4
#define NON_MODAL_SET_COORDINATE_DATA 2 // G10
#define NON_MODAL_GO_HOME_0 3 // G28
#define NON_MODAL_SET_HOME_0 4 // G28.1
#define NON_MODAL_GO_HOME_1 5 // G30
#define NON_MODAL_SET_HOME_1 6 // G30.1
#define NON_MODAL_SET_COORDINATE_OFFSET 7 // G92
#define NON_MODAL_RESET_COORDINATE_OFFSET 8 //G92.1

typedef struct {
  uint8_t status_code;             // Parser status for current block
  uint8_t motion_mode;             // {G0, G1, G2, G3, G80}
  uint8_t inverse_feed_rate_mode;  // {G93, G94}
  uint8_t inches_mode;             // 0 = millimeter mode, 1 = inches mode {G20, G21}
  uint8_t absolute_mode;           // 0 = relative motion, 1 = absolute motion {G90, G91}
  uint8_t program_flow;            // {M0, M1, M2, M30}
  uint8_t coolant_mode;            // 0 = Disable, 1 = Flood Enable, 2 = Mist Enable {M8, M9}
  uint8_t spindle_direction;        // 1 = CW, 2 = CCW, 0 = Stop {M3, M4, M5}
  float spindle_speed;             // RPM
  float feed_rate;                 // Millimeters/min
  float position[N_AXIS];          // Where the interpreter considers the tool to be at this point in the code
  uint8_t tool;
  uint8_t plane_axis_0, 
          plane_axis_1, 
          plane_axis_2;            // The axes of the selected plane  
  uint8_t coord_select;            // Active work coordinate system number. Default: 0=G54.
  float coord_system[N_AXIS];      // Current work coordinate system (G54+). Stores offset from absolute machine
                                   // position in mm. Loaded from EEPROM when called.
  float coord_offset[N_AXIS];      // Retains the G92 coordinate offset (work coordinates) relative to
                                   // machine zero in mm. Non-persistent. Cleared upon reset and boot.    
                                   
  float arc_radius; 
  float arc_offset[N_AXIS];
                                         
} parser_state_t;
extern parser_state_t gc;

// Initialize the parser
void gc_init();

// Execute one block of rs275/ngc/g-code
uint8_t gc_execute_line(char *line);

// Set g-code parser position. Input in steps.
void gc_sync_position(); 

#endif
