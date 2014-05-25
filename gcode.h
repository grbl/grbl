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
// NOTE: Modal group define values must be sequential and starting from zero.
#define MODAL_GROUP_G0 0 // [G4,G10,G28,G28.1,G30,G30.1,G53,G92,G92.1] Non-modal
#define MODAL_GROUP_G1 1 // [G0,G1,G2,G3,G38.2,G80] Motion
#define MODAL_GROUP_G2 2 // [G17,G18,G19] Plane selection
#define MODAL_GROUP_G3 3 // [G90,G91] Distance mode
#define MODAL_GROUP_G5 4 // [G93,G94] Feed rate mode
#define MODAL_GROUP_G6 5 // [G20,G21] Units
#define MODAL_GROUP_G12 6 // [G54,G55,G56,G57,G58,G59] Coordinate system selection

#define MODAL_GROUP_M4 7 // [M0,M1,M2,M30] Stopping
#define MODAL_GROUP_M7 8 // [M3,M4,M5] Spindle turning
#define MODAL_GROUP_M8 9 // [M7,M8,M9] Coolant control

#define OTHER_INPUT_F 10
#define OTHER_INPUT_S 11
#define OTHER_INPUT_T 12

// Define command actions for within execution-type modal groups (motion, stopping, non-modal). Used
// internally by the parser to know which command to execute.

// Modal Group 0: Non-modal actions
#define NON_MODAL_NO_ACTION 0 // (Default: Must be zero)
#define NON_MODAL_DWELL 1 // G4
#define NON_MODAL_SET_COORDINATE_DATA 2 // G10
#define NON_MODAL_GO_HOME_0 3 // G28
#define NON_MODAL_SET_HOME_0 4 // G28.1
#define NON_MODAL_GO_HOME_1 5 // G30
#define NON_MODAL_SET_HOME_1 6 // G30.1
#define NON_MODAL_ABSOLUTE_OVERRIDE 7 // G53
#define NON_MODAL_SET_COORDINATE_OFFSET 8 // G92
#define NON_MODAL_RESET_COORDINATE_OFFSET 9 //G92.1

// Modal Group 1: Motion modes
#define MOTION_MODE_SEEK 0 // G0 (Default: Must be zero)
#define MOTION_MODE_LINEAR 1 // G1
#define MOTION_MODE_CW_ARC 2  // G2
#define MOTION_MODE_CCW_ARC 3  // G3
#define MOTION_MODE_PROBE 4 // G38.2
#define MOTION_MODE_NONE 5 // G80

// Modal Group 2: Plane select
#define PLANE_SELECT_XY 0 // G17 (Default: Must be zero)
#define PLANE_SELECT_ZX 1 // G18
#define PLANE_SELECT_YZ 2 // G19

// Modal Group 3: Distance mode
#define DISTANCE_MODE_ABSOLUTE 0 // G90 (Default: Must be zero)
#define DISTANCE_MODE_INCREMENTAL 1 // G91

// Modal Group 4: Program flow
#define PROGRAM_FLOW_RUNNING 0 // (Default: Must be zero)
#define PROGRAM_FLOW_PAUSED 1 // M0, M1
#define PROGRAM_FLOW_COMPLETED 2 // M2, M30

// Modal Group 5: Feed rate mode
#define FEED_RATE_MODE_UNITS_PER_MIN 0 // G94 (Default: Must be zero)
#define FEED_RATE_MODE_INVERSE_TIME 1 // G93

// Modal Group 6: Units mode
#define UNITS_MODE_MM 0 // G21 (Default: Must be zero)
#define UNITS_MODE_INCHES 1 // G20

// Modal Group 7: Spindle control
#define SPINDLE_DISABLE 0 // M5 (Default: Must be zero)
#define SPINDLE_ENABLE_CW 1 // M3
#define SPINDLE_ENABLE_CCW 2 // M4

// Modal Group 8: Coolant control
#define COOLANT_DISABLE 0 // M9 (Default: Must be zero)
#define COOLANT_MIST_ENABLE 1 // M7
#define COOLANT_FLOOD_ENABLE 2 // M8

// Modal Group 12: Active work coordinate system
// N/A: Stores coordinate system value (54-59) to change to.

#define WORD_F  0
#define WORD_I  1
#define WORD_J  2
#define WORD_K  3
#define WORD_L  4
#define WORD_N  5
#define WORD_P  6
#define WORD_R  7
#define WORD_S  8
#define WORD_T  9
#define WORD_X  10
#define WORD_Y  11
#define WORD_Z  12




// NOTE: When this struct is zeroed, the above defines set the defaults for the system.
typedef struct {
  uint8_t motion;        // {G0,G1,G2,G3,G80}
  uint8_t feed_rate;     // {G93,G94}
  uint8_t units;         // {G20,G21}
  uint8_t distance;      // {G90,G91}
  uint8_t plane_select;  // {G17,G18,G19}
  uint8_t coord_select;  // {G54,G55,G56,G57,G58,G59}
  uint8_t program_flow;  // {M0,M1,M2,M30}
  uint8_t coolant;       // {M7,M8,M9}
  uint8_t spindle;       // {M3,M4,M5}
} gc_modal_t;  

typedef struct {
  float f;         // Feed
  float ijk[3];    // I,J,K Axis arc offsets
  uint8_t l;       // G10 or canned cycles parameters
  int32_t n;       // Line number
  float p;         // G10 or dwell parameters
  // float q;      // G82 peck drilling
  float r;         // Arc radius
  float s;         // Spindle speed
  // uint8_t t;    // Tool selection
  float xyz[3];    // X,Y,Z Translational axes
} gc_values_t;


typedef struct {
  gc_modal_t modal;
  
  float spindle_speed;             // RPM
  float feed_rate;                 // Millimeters/min
  uint8_t tool;

  float position[N_AXIS];          // Where the interpreter considers the tool to be at this point in the code

  float coord_system[N_AXIS];      // Current work coordinate system (G54+). Stores offset from absolute machine
                                   // position in mm. Loaded from EEPROM when called.  
  float coord_offset[N_AXIS];      // Retains the G92 coordinate offset (work coordinates) relative to
                                   // machine zero in mm. Non-persistent. Cleared upon reset and boot.    
} parser_state_t;
extern parser_state_t gc_state;

typedef struct {
//   uint16_t command_words;  // NOTE: If this bitflag variable fills, G and M words can be separated.
//   uint16_t value_words;

  uint8_t non_modal_command;
  gc_modal_t modal;
  gc_values_t values;

} parser_block_t;
extern parser_block_t gc_block;

// Initialize the parser
void gc_init();

// Execute one block of rs275/ngc/g-code
uint8_t gc_execute_line(char *line);

// Set g-code parser position. Input in steps.
void gc_sync_position(); 

#endif
