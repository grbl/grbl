/*
  gcode.c - rs274/ngc parser.
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

/* This code is inspired by the Arduino GCode Interpreter by Mike Ellery and the NIST RS274/NGC Interpreter
   by Kramer, Proctor and Messina. */

#include "gcode.h"
#include <string.h>
#include "nuts_bolts.h"
#include <math.h>
#include "settings.h"
#include "motion_control.h"
#include "spindle_control.h"
#include "errno.h"
#include "protocol.h"

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
#define NON_MODAL_GO_HOME 3 // G28,G30
#define NON_MODAL_SET_COORDINATE_OFFSET 4 // G92
#define NON_MODAL_RESET_COORDINATE_OFFSET 5 //G92.1

typedef struct {
  uint8_t status_code;             // Parser status for current block
  uint8_t motion_mode;             // {G0, G1, G2, G3, G80}
  uint8_t inverse_feed_rate_mode;  // {G93, G94}
  uint8_t inches_mode;             // 0 = millimeter mode, 1 = inches mode {G20, G21}
  uint8_t absolute_mode;           // 0 = relative motion, 1 = absolute motion {G90, G91}
  uint8_t program_flow;            // {M0, M1, M2, M30}
  int8_t spindle_direction;        // 1 = CW, -1 = CCW, 0 = Stop {M3, M4, M5}
  double feed_rate, seek_rate;     // Millimeters/second
  double position[3];              // Where the interpreter considers the tool to be at this point in the code
  uint8_t tool;
  int16_t spindle_speed;           // RPM/100
  uint8_t plane_axis_0, 
          plane_axis_1, 
          plane_axis_2;            // The axes of the selected plane  
} parser_state_t;
static parser_state_t gc;

#define FAIL(status) gc.status_code = status;

static int next_statement(char *letter, double *double_ptr, char *line, uint8_t *char_counter);

static void select_plane(uint8_t axis_0, uint8_t axis_1, uint8_t axis_2) 
{
  gc.plane_axis_0 = axis_0;
  gc.plane_axis_1 = axis_1;
  gc.plane_axis_2 = axis_2;
}

void gc_init() 
{
  memset(&gc, 0, sizeof(gc));
  gc.feed_rate = settings.default_feed_rate;
  select_plane(X_AXIS, Y_AXIS, Z_AXIS);
  gc.absolute_mode = true;
}

// Sets g-code parser position in mm. Input in steps. Called by the system abort routine.
void gc_set_current_position(int32_t x, int32_t y, int32_t z) 
{
  gc.position[X_AXIS] = x/settings.steps_per_mm[X_AXIS];
  gc.position[Y_AXIS] = y/settings.steps_per_mm[Y_AXIS];
  gc.position[Z_AXIS] = z/settings.steps_per_mm[Z_AXIS]; 
}

static float to_millimeters(double value) 
{
  return(gc.inches_mode ? (value * MM_PER_INCH) : value);
}

// Executes one line of 0-terminated G-Code. The line is assumed to contain only uppercase
// characters and signed floating point values (no whitespace). Comments and block delete
// characters have been removed. All units and positions are converted and exported to grbl's
// internal functions in terms of (mm, mm/min) and absolute machine coordinates, respectively.
uint8_t gc_execute_line(char *line) 
{
  uint8_t char_counter = 0;  
  char letter;
  double value;
  int int_value;
  
  uint16_t modal_group_words = 0;  // Bitflag variable to track and check modal group words in block
  uint8_t axis_words = 0;          // Bitflag to track which XYZ(ABC) parameters exist in block

  double inverse_feed_rate = -1; // negative inverse_feed_rate means no inverse_feed_rate specified
  uint8_t absolute_override = false; // true(1) = absolute motion for this block only {G53}
  uint8_t non_modal_action = NON_MODAL_NONE; // Tracks the actions of modal group 0 (non-modal)
  
  double target[3], offset[3];  
  clear_vector(target); // XYZ(ABC) axes parameters.
  clear_vector(offset); // IJK Arc offsets are incremental. Value of zero indicates no change.
    
  gc.status_code = STATUS_OK;
  
  /* Pass 1: Commands and set all modes. Check for modal group violations.
     NOTE: Modal group numbers are defined in Table 4 of NIST RS274-NGC v3, pg.20 */
  uint8_t group_number = MODAL_GROUP_NONE;
  while(next_statement(&letter, &value, line, &char_counter)) {
    int_value = trunc(value);
    switch(letter) {
      case 'G':
        // Set modal group values
        switch(int_value) {
          case 4: case 10: case 28: case 30: case 53: case 92: group_number = MODAL_GROUP_0; break;
          case 0: case 1: case 2: case 3: case 80: group_number = MODAL_GROUP_1; break;
          case 17: case 18: case 19: group_number = MODAL_GROUP_2; break;
          case 90: case 91: group_number = MODAL_GROUP_3; break;
          case 93: case 94: group_number = MODAL_GROUP_5; break;
          case 20: case 21: group_number = MODAL_GROUP_6; break;
          case 54: case 55: case 56: case 57: case 58: case 59: group_number = MODAL_GROUP_12; break;
        }          
        // Set 'G' commands
        switch(int_value) {
          case 0: gc.motion_mode = MOTION_MODE_SEEK; break;
          case 1: gc.motion_mode = MOTION_MODE_LINEAR; break;
          case 2: gc.motion_mode = MOTION_MODE_CW_ARC; break;
          case 3: gc.motion_mode = MOTION_MODE_CCW_ARC; break;
          case 4: non_modal_action = NON_MODAL_DWELL; break;
          case 10: non_modal_action = NON_MODAL_SET_COORDINATE_DATA; break;
          case 17: select_plane(X_AXIS, Y_AXIS, Z_AXIS); break;
          case 18: select_plane(X_AXIS, Z_AXIS, Y_AXIS); break;
          case 19: select_plane(Y_AXIS, Z_AXIS, X_AXIS); break;
          case 20: gc.inches_mode = true; break;
          case 21: gc.inches_mode = false; break;
          case 28: case 30: non_modal_action = NON_MODAL_GO_HOME; break;
          case 53: absolute_override = true; break;
          case 54: case 55: case 56: case 57: case 58: case 59:
            int_value -= 54; // Compute coordinate system row index (0=G54,1=G55,...)
            if (int_value < N_COORDINATE_SYSTEM) {
              sys.coord_select = int_value;
            } else {
              FAIL(STATUS_UNSUPPORTED_STATEMENT);
            }
            break;
          case 80: gc.motion_mode = MOTION_MODE_CANCEL; break;
          case 90: gc.absolute_mode = true; break;
          case 91: gc.absolute_mode = false; break;
          case 92: 
            int_value = trunc(10*value); // Multiply by 10 to pick up G92.1
            switch(int_value) {
              case 920: non_modal_action = NON_MODAL_SET_COORDINATE_OFFSET; break;        
              case 921: non_modal_action = NON_MODAL_RESET_COORDINATE_OFFSET; break;
              default: FAIL(STATUS_UNSUPPORTED_STATEMENT);
            }
            break;
          case 93: gc.inverse_feed_rate_mode = true; break;
          case 94: gc.inverse_feed_rate_mode = false; break;
          default: FAIL(STATUS_UNSUPPORTED_STATEMENT);
        }
        break;        
      case 'M':
        // Set modal group values
        switch(int_value) {
          case 0: case 1: case 2: case 30: group_number = MODAL_GROUP_4; break;
          case 3: case 4: case 5: group_number = MODAL_GROUP_7; break;
        }        
        // Set 'M' commands
        switch(int_value) {
          case 0: gc.program_flow = PROGRAM_FLOW_PAUSED; break; // Program pause
          case 1: // Program pause with optional stop on
            // if (sys.opt_stop) { // TODO: Add system variable for optional stop.
            gc.program_flow = PROGRAM_FLOW_PAUSED; break; 
            // }
          case 2: case 30: gc.program_flow = PROGRAM_FLOW_COMPLETED; break; // Program end and reset 
          case 3: gc.spindle_direction = 1; break;
          case 4: gc.spindle_direction = -1; break;
          case 5: gc.spindle_direction = 0; break;
          default: FAIL(STATUS_UNSUPPORTED_STATEMENT);
        }            
        break;
    }    
    // Check for modal group multiple command violations in the current block
    if (group_number) {
      if ( bit_istrue(modal_group_words,bit(group_number)) ) {
        FAIL(STATUS_MODAL_GROUP_VIOLATION);
      } else {
        bit_true(modal_group_words,bit(group_number));
      }
      group_number = MODAL_GROUP_NONE; // Reset for next command.
    }
  } 

  // If there were any errors parsing this line, we will return right away with the bad news
  if (gc.status_code) { return(gc.status_code); }
  
  /* Pass 2: Parameters. All units converted according to current block commands. Position 
     parameters are converted and flagged to indicate a change. These can have multiple connotations
     for different commands. Each will be converted to their proper value upon execution. */
  double p = 0, r = 0;
  uint8_t l = 0;
  char_counter = 0;
  while(next_statement(&letter, &value, line, &char_counter)) {
    switch(letter) {
      case 'F': 
        if (value <= 0) { FAIL(STATUS_INVALID_COMMAND); } // Must be greater than zero
        if (gc.inverse_feed_rate_mode) {
          inverse_feed_rate = to_millimeters(value); // seconds per motion for this motion only
        } else {          
          gc.feed_rate = to_millimeters(value); // millimeters per minute
        }
        break;
      case 'I': case 'J': case 'K': offset[letter-'I'] = to_millimeters(value); break;
      case 'L': l = trunc(value); break;
      case 'P': p = value; break;                    
      case 'R': r = to_millimeters(value); break;
      case 'S': 
        if (value < 0) { FAIL(STATUS_INVALID_COMMAND); } // Cannot be negative
        gc.spindle_speed = value;
        break;
      case 'T': 
        if (value < 0) { FAIL(STATUS_INVALID_COMMAND); } // Cannot be negative
        gc.tool = trunc(value); 
        break;
      case 'X': target[X_AXIS] = to_millimeters(value); bit_true(axis_words,bit(X_AXIS)); break;
      case 'Y': target[Y_AXIS] = to_millimeters(value); bit_true(axis_words,bit(Y_AXIS)); break;
      case 'Z': target[Z_AXIS] = to_millimeters(value); bit_true(axis_words,bit(Z_AXIS)); break;
    }
  }
  
  // If there were any errors parsing this line, we will return right away with the bad news
  if (gc.status_code) { return(gc.status_code); }
  
  
  /* Execute Commands: Perform by order of execution defined in NIST RS274-NGC.v3, Table 8, pg.41.
     NOTE: Independent non-motion/settings parameters are set out of this order for code efficiency 
     and simplicity purposes, but this should not affect proper g-code execution. */
  
  //  ([M6]: Tool change execution should be executed here.)
  
  // [M3,M4,M5]: Update spindle state
  spindle_run(gc.spindle_direction, gc.spindle_speed);
  
  //  ([M7,M8,M9]: Coolant state should be executed here.)
  
  // [G4,G10,G28,G30,G92,G92.1]: Perform dwell, set coordinate system data, homing, or set axis offsets.
  // NOTE: These commands are in the same modal group, hence are mutually exclusive. G53 is in this
  // modal group and do not effect these actions.
  switch (non_modal_action) {
    case NON_MODAL_DWELL:
      if (p < 0) { // Time cannot be negative.
        FAIL(STATUS_INVALID_COMMAND); 
      } else { 
        mc_dwell(p); 
      }
      break;
    case NON_MODAL_SET_COORDINATE_DATA:
      int_value = trunc(p); // Convert p value to int.
      if (l != 2 || (int_value < 1 || int_value > N_COORDINATE_SYSTEM)) { // L2 only. P1=G54, P2=G55, ... 
        FAIL(STATUS_UNSUPPORTED_STATEMENT); 
      } else if (!axis_words) { // No axis words.
        FAIL(STATUS_INVALID_COMMAND);
      } else {
        int_value--; // Adjust p to be inline with row array index. 
        // Update axes defined only in block. Always in machine coordinates. Can change non-active system.
        uint8_t i;
        for (i=0; i<=2; i++) { // Axes indices are consistent, so loop may be used.
          if ( bit_istrue(axis_words,bit(i)) ) { sys.coord_system[int_value][i] = target[i]; }
        }
      }
      axis_words = 0; // Axis words used. Lock out from motion modes by clearing flags.
      break;
    case NON_MODAL_GO_HOME: 
      // Move to intermediate position before going home. Obeys current coordinate system and offsets 
      // and absolute and incremental modes.
      if (axis_words) {
        // Apply absolute mode coordinate offsets or incremental mode offsets.
        uint8_t i;
        for (i=0; i<=2; i++) { // Axes indices are consistent, so loop may be used.
          if ( bit_istrue(axis_words,bit(i)) ) {
            if (gc.absolute_mode) {
              target[i] += sys.coord_system[sys.coord_select][i] + sys.coord_offset[i];
            } else {
              target[i] += gc.position[i];
            }
          } else {
            target[i] = gc.position[i];
          }
        }
        mc_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], settings.default_seek_rate, false);
      }
      mc_go_home(); 
      clear_vector(gc.position); // Assumes home is at [0,0,0]
      axis_words = 0; // Axis words used. Lock out from motion modes by clearing flags.
      break;      
    case NON_MODAL_SET_COORDINATE_OFFSET:
      if (!axis_words) { // No axis words
        FAIL(STATUS_INVALID_COMMAND);
      } else {
        // Update axes defined only in block. Offsets current system to defined value. Does not update when
        // active coordinate system is selected, but is still active unless G92.1 disables it. 
        uint8_t i;
        for (i=0; i<=2; i++) { // Axes indices are consistent, so loop may be used.
          if (bit_istrue(axis_words,bit(i)) ) {
            sys.coord_offset[i] = gc.position[i]-sys.coord_system[sys.coord_select][i]-target[i];
          }
        }
      }
      axis_words = 0; // Axis words used. Lock out from motion modes by clearing flags.
      break;
    case NON_MODAL_RESET_COORDINATE_OFFSET: 
      clear_vector(sys.coord_offset); // Disable G92 offsets by zeroing offset vector.
      break;
  }

  // [G0,G1,G2,G3,G80]: Perform motion modes. 
  // NOTE: Commands G10,G28,G30,G92 lock out and prevent axis words from use in motion modes. 
  // Enter motion modes only if there are axis words or a motion mode command word in the block.
  if ( bit_istrue(modal_group_words,bit(MODAL_GROUP_1)) || axis_words ) {

    // G1,G2,G3 require F word in inverse time mode.  
    if ( gc.inverse_feed_rate_mode ) { 
      if (inverse_feed_rate < 0 && gc.motion_mode != MOTION_MODE_CANCEL) {
        FAIL(STATUS_INVALID_COMMAND);
      }
    }
    // Absolute override G53 only valid with G0 and G1 active.
    if ( absolute_override && !(gc.motion_mode == MOTION_MODE_SEEK || gc.motion_mode == MOTION_MODE_LINEAR)) {
      FAIL(STATUS_INVALID_COMMAND);
    }
    // Report any errors.  
    if (gc.status_code) { return(gc.status_code); }

    // Convert all target position data to machine coordinates for executing motion. Apply
    // absolute mode coordinate offsets or incremental mode offsets.
    // NOTE: Tool offsets may be appended to these conversions when/if this feature is added.
    uint8_t i;
    for (i=0; i<=2; i++) { // Axes indices are consistent, so loop may be used to save flash space.
      if ( bit_istrue(axis_words,bit(i)) ) {
        if (!absolute_override) { // Do not update target in absolute override mode
          if (gc.absolute_mode) {
            target[i] += sys.coord_system[sys.coord_select][i] + sys.coord_offset[i]; // Absolute mode
          } else {
            target[i] += gc.position[i]; // Incremental mode
          }
        }
      } else {
        target[i] = gc.position[i]; // No axis word in block. Keep same axis position.
      }
    }
  
    switch (gc.motion_mode) {
      case MOTION_MODE_CANCEL: 
        if (axis_words) { FAIL(STATUS_INVALID_COMMAND); } // No axis words allowed while active.
        break;
      case MOTION_MODE_SEEK:
        if (!axis_words) { FAIL(STATUS_INVALID_COMMAND);} 
        else { mc_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], settings.default_seek_rate, false); }
        break;
      case MOTION_MODE_LINEAR:
        if (!axis_words) { FAIL(STATUS_INVALID_COMMAND);} 
        else { mc_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], 
          (gc.inverse_feed_rate_mode) ? inverse_feed_rate : gc.feed_rate, gc.inverse_feed_rate_mode); }
        break;
      case MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC:
        // Check if at least one of the axes of the selected plane has been specified. If in center 
        // format arc mode, also check for at least one of the IJK axes of the selected plane was sent.
        if ( !( bit_false(axis_words,bit(gc.plane_axis_2)) ) || 
             ( !r && !offset[gc.plane_axis_0] && !offset[gc.plane_axis_1] ) ) { 
          FAIL(STATUS_INVALID_COMMAND);
        } else {
          if (r != 0) { // Arc Radius Mode
            /* 
              We need to calculate the center of the circle that has the designated radius and passes
              through both the current position and the target position. This method calculates the following
              set of equations where [x,y] is the vector from current to target position, d == magnitude of 
              that vector, h == hypotenuse of the triangle formed by the radius of the circle, the distance to
              the center of the travel vector. A vector perpendicular to the travel vector [-y,x] is scaled to the 
              length of h [-y/d*h, x/d*h] and added to the center of the travel vector [x/2,y/2] to form the new point 
              [i,j] at [x/2-y/d*h, y/2+x/d*h] which will be the center of our arc.
              
              d^2 == x^2 + y^2
              h^2 == r^2 - (d/2)^2
              i == x/2 - y/d*h
              j == y/2 + x/d*h
              
                                                                   O <- [i,j]
                                                                -  |
                                                      r      -     |
                                                          -        |
                                                       -           | h
                                                    -              |
                                      [0,0] ->  C -----------------+--------------- T  <- [x,y]
                                                | <------ d/2 ---->|
                        
              C - Current position
              T - Target position
              O - center of circle that pass through both C and T
              d - distance from C to T
              r - designated radius
              h - distance from center of CT to O
              
              Expanding the equations:
    
              d -> sqrt(x^2 + y^2)
              h -> sqrt(4 * r^2 - x^2 - y^2)/2
              i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2 
              j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
             
              Which can be written:
              
              i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
              j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
              
              Which we for size and speed reasons optimize to:
    
              h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
              i = (x - (y * h_x2_div_d))/2
              j = (y + (x * h_x2_div_d))/2
              
            */
            
            // Calculate the change in position along each selected axis
            double x = target[gc.plane_axis_0]-gc.position[gc.plane_axis_0];
            double y = target[gc.plane_axis_1]-gc.position[gc.plane_axis_1];
            
            clear_vector(offset);
            double h_x2_div_d = -sqrt(4 * r*r - x*x - y*y)/hypot(x,y); // == -(h * 2 / d)
            // If r is smaller than d, the arc is now traversing the complex plane beyond the reach of any
            // real CNC, and thus - for practical reasons - we will terminate promptly:
            if(isnan(h_x2_div_d)) { FAIL(STATUS_FLOATING_POINT_ERROR); return(gc.status_code); }
            // Invert the sign of h_x2_div_d if the circle is counter clockwise (see sketch below)
            if (gc.motion_mode == MOTION_MODE_CCW_ARC) { h_x2_div_d = -h_x2_div_d; }
            
            /* The counter clockwise circle lies to the left of the target direction. When offset is positive,
               the left hand circle will be generated - when it is negative the right hand circle is generated.
               
               
                                                             T  <-- Target position
                                                             
                                                             ^ 
                  Clockwise circles with this center         |          Clockwise circles with this center will have
                  will have > 180 deg of angular travel      |          < 180 deg of angular travel, which is a good thing!
                                                   \         |          /   
      center of arc when h_x2_div_d is positive ->  x <----- | -----> x <- center of arc when h_x2_div_d is negative
                                                             |
                                                             |
                                                             
                                                             C  <-- Current position                                 */
                    
    
            // Negative R is g-code-alese for "I want a circle with more than 180 degrees of travel" (go figure!), 
            // even though it is advised against ever generating such circles in a single line of g-code. By 
            // inverting the sign of h_x2_div_d the center of the circles is placed on the opposite side of the line of
            // travel and thus we get the unadvisably long arcs as prescribed.
            if (r < 0) { 
                h_x2_div_d = -h_x2_div_d; 
                r = -r; // Finished with r. Set to positive for mc_arc
            }        
            // Complete the operation by calculating the actual center of the arc
            offset[gc.plane_axis_0] = 0.5*(x-(y*h_x2_div_d));
            offset[gc.plane_axis_1] = 0.5*(y+(x*h_x2_div_d));

          } else { // Arc Center Format Offset Mode            
            r = hypot(offset[gc.plane_axis_0], offset[gc.plane_axis_1]); // Compute arc radius for mc_arc
          }
          
          // Set clockwise/counter-clockwise sign for mc_arc computations
          uint8_t isclockwise = false;
          if (gc.motion_mode == MOTION_MODE_CW_ARC) { isclockwise = true; }
    
          // Trace the arc
          mc_arc(gc.position, target, offset, gc.plane_axis_0, gc.plane_axis_1, gc.plane_axis_2,
            (gc.inverse_feed_rate_mode) ? inverse_feed_rate : gc.feed_rate, gc.inverse_feed_rate_mode,
            r, isclockwise);
        }            
        break;
    }
    
    // Report any errors.
    if (gc.status_code) { return(gc.status_code); }    
    
    // As far as the parser is concerned, the position is now == target. In reality the
    // motion control system might still be processing the action and the real tool position
    // in any intermediate location.
    memcpy(gc.position, target, sizeof(double)*3); // gc.position[] = target[];
  }
  
  // M0,M1,M2,M30: Perform non-running program flow actions. During a program pause, the buffer may 
  // refill and can only be resumed by the cycle start run-time command.
  if (gc.program_flow) {
    plan_synchronize(); // Finish all remaining buffered motions. Program paused when complete.
    sys.auto_start = false; // Disable auto cycle start.
    gc.program_flow = PROGRAM_FLOW_RUNNING; // Re-enable program flow after pause complete.
    
    // If complete, reset to reload defaults (G92.2,G54,G17,G90,G94,M48,G40,M5,M9)
    if (gc.program_flow == PROGRAM_FLOW_COMPLETED) { sys.abort = true; }
  }    
  
  return(gc.status_code);
}

// Parses the next statement and leaves the counter on the first character following
// the statement. Returns 1 if there was a statements, 0 if end of string was reached
// or there was an error (check state.status_code).
static int next_statement(char *letter, double *double_ptr, char *line, uint8_t *char_counter) 
{
  if (line[*char_counter] == 0) {
    return(0); // No more statements
  }
  
  *letter = line[*char_counter];
  if((*letter < 'A') || (*letter > 'Z')) {
    FAIL(STATUS_EXPECTED_COMMAND_LETTER);
    return(0);
  }
  (*char_counter)++;
  if (!read_double(line, char_counter, double_ptr)) {
    FAIL(STATUS_BAD_NUMBER_FORMAT); 
    return(0);
  };
  return(1);
}

/* 
  Not supported:

  - Canned cycles
  - Tool radius compensation
  - A,B,C-axes
  - Evaluation of expressions
  - Variables
  - Multiple home locations
  - Multiple coordinate systems (Up to 6 may be added via config.h)
  - Probing
  - Override control
  - Tool changes

   group 0 = {G92.2, G92.3} (Non modal: Cancel and re-enable G92 offsets)
   group 1 = {G38.2, G81 - G89} (Motion modes: straight probe, canned cycles)
   group 6 = {M6} (Tool change)
   group 8 = {M7, M8, M9} coolant (special case: M7 and M8 may be active at the same time)
   group 9 = {M48, M49} enable/disable feed and speed override switches
   group 12 = {G55, G56, G57, G58, G59, G59.1, G59.2, G59.3} coordinate system selection
   group 13 = {G61, G61.1, G64} path control mode
*/
