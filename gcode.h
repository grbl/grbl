/*
  gcode.h - rs274/ngc parser.
  Part of Grbl

  The MIT License (MIT)

  GRBL(tm) - Embedded CNC g-code interpreter and motion-controller
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011-2012 Sungeun K. Jeon

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#ifndef gcode_h
#define gcode_h
#include <avr/io.h>
#include "nuts_bolts.h"

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
#define MODAL_GROUP_12 9 // [G54,G55,G56,G57,G58,G59] Coordinate system selection

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
  int8_t spindle_direction;        // 1 = CW, -1 = CCW, 0 = Stop {M3, M4, M5}
  uint8_t coolant_mode;            // 0 = Disable, 1 = Flood Enable {M8, M9}
  float feed_rate;                 // Millimeters/min
//  float seek_rate;                 // Millimeters/min. Will be used in v0.9 when axis independence is installed
  float position[3];               // Where the interpreter considers the tool to be at this point in the code
  uint8_t tool;
//  uint16_t spindle_speed;          // RPM/100
  uint8_t plane_axis_0, 
          plane_axis_1, 
          plane_axis_2;            // The axes of the selected plane  
  uint8_t coord_select;            // Active work coordinate system number. Default: 0=G54.
  float coord_system[N_AXIS];      // Current work coordinate system (G54+). Stores offset from absolute machine
                                   // position in mm. Loaded from EEPROM when called.
  float coord_offset[N_AXIS];      // Retains the G92 coordinate offset (work coordinates) relative to
                                   // machine zero in mm. Non-persistent. Cleared upon reset and boot.        
} parser_state_t;
extern parser_state_t gc;

// Initialize the parser
void gc_init();

// Execute one block of rs275/ngc/g-code
uint8_t gc_execute_line(char *line);

// Set g-code parser position. Input in steps.
void gc_set_current_position(int32_t x, int32_t y, int32_t z); 

#endif
