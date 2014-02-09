/*
  gcode.c - rs274/ngc parser.
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

/* This code is inspired by the Arduino GCode Interpreter by Mike Ellery and the NIST RS274/NGC Interpreter
   by Kramer, Proctor and Messina. */

#include "system.h"
#include "settings.h"
#include "protocol.h"
#include "gcode.h"
#include "motion_control.h"
#include "spindle_control.h"
#include "coolant_control.h"
#include "report.h"

// Declare gc extern struct
parser_state_t gc;

#define FAIL(status) gc.status_code = status;

static uint8_t next_statement(char *letter, float *float_ptr, char *line, uint8_t *char_counter);
static void gc_convert_arc_radius_mode(float *target) __attribute__((noinline));


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
  
  // Load default G54 coordinate system.
  if (!(settings_read_coord_data(gc.coord_select,gc.coord_system))) { 
    report_status_message(STATUS_SETTING_READ_FAIL); 
  } 
}


// Sets g-code parser position in mm. Input in steps. Called by the system abort and hard
// limit pull-off routines.
void gc_sync_position(int32_t x, int32_t y, int32_t z) 
{
  uint8_t i;
  for (i=0; i<N_AXIS; i++) {
    gc.position[i] = sys.position[i]/settings.steps_per_mm[i];
  }
}


static float to_millimeters(float value) 
{
  return(gc.inches_mode ? (value * MM_PER_INCH) : value);
}

         
// Executes one line of 0-terminated G-Code. The line is assumed to contain only uppercase
// characters and signed floating point values (no whitespace). Comments and block delete
// characters have been removed. In this function, all units and positions are converted and 
// exported to grbl's internal functions in terms of (mm, mm/min) and absolute machine 
// coordinates, respectively.
uint8_t gc_execute_line(char *line) 
{
  uint8_t char_counter = 0;  
  char letter;
  float value;
  int int_value;
  
  uint16_t modal_group_words = 0;  // Bitflag variable to track and check modal group words in block
  uint8_t axis_words = 0;          // Bitflag to track which XYZ(ABC) parameters exist in block

  float inverse_feed_rate = -1; // negative inverse_feed_rate means no inverse_feed_rate specified
  uint8_t absolute_override = false; // true(1) = absolute motion for this block only {G53}
  uint8_t non_modal_action = NON_MODAL_NONE; // Tracks the actions of modal group 0 (non-modal)
  
  float target[N_AXIS];
  clear_vector(target); // XYZ(ABC) axes parameters.

  gc.arc_radius = 0;
  clear_vector(gc.arc_offset); // IJK Arc offsets are incremental. Value of zero indicates no change.

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
          case 18: select_plane(Z_AXIS, X_AXIS, Y_AXIS); break;
          case 19: select_plane(Y_AXIS, Z_AXIS, X_AXIS); break;
          case 20: gc.inches_mode = true; break;
          case 21: gc.inches_mode = false; break;
          case 28: case 30: 
            int_value = trunc(10*value); // Multiply by 10 to pick up Gxx.1
            switch(int_value) {
              case 280: non_modal_action = NON_MODAL_GO_HOME_0; break;
              case 281: non_modal_action = NON_MODAL_SET_HOME_0; break;
              case 300: non_modal_action = NON_MODAL_GO_HOME_1; break;
              case 301: non_modal_action = NON_MODAL_SET_HOME_1; break;
              default: FAIL(STATUS_UNSUPPORTED_STATEMENT);
            }
            break;
          case 53: absolute_override = true; break;
          case 54: case 55: case 56: case 57: case 58: case 59:
            gc.coord_select = int_value-54;
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
          case 7: case 8: case 9: group_number = MODAL_GROUP_8; break;
        }        
        // Set 'M' commands
        switch(int_value) {
          case 0: gc.program_flow = PROGRAM_FLOW_PAUSED; break; // Program pause
          case 1: break; // Optional stop not supported. Ignore.
          case 2: case 30: gc.program_flow = PROGRAM_FLOW_COMPLETED; break; // Program end and reset 
          case 3: gc.spindle_direction = SPINDLE_ENABLE_CW; break;
          case 4: gc.spindle_direction = SPINDLE_ENABLE_CCW; break;
          case 5: gc.spindle_direction = SPINDLE_DISABLE; break;
          #ifdef ENABLE_M7
            case 7: gc.coolant_mode = COOLANT_MIST_ENABLE; break;
          #endif
          case 8: gc.coolant_mode = COOLANT_FLOOD_ENABLE; break;
          case 9: gc.coolant_mode = COOLANT_DISABLE; break;
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
     for different commands. Each will be converted to their proper value upon execution. 
     NOTE: Grbl unconventionally pre-converts these parameter values based on the block G and M 
     commands. This is set out of the order of execution defined by NIST only for code efficiency/size 
     purposes, but should not affect proper g-code execution. */ 
  float p = 0;
  uint8_t l = 0;
  char_counter = 0;
  while(next_statement(&letter, &value, line, &char_counter)) {
    switch(letter) {
      case 'G': case 'M': case 'N': break; // Ignore command statements and line numbers
      case 'F': 
        if (value <= 0) { FAIL(STATUS_INVALID_STATEMENT); } // Must be greater than zero
        if (gc.inverse_feed_rate_mode) {
          inverse_feed_rate = to_millimeters(value); // seconds per motion for this motion only
        } else {          
          gc.feed_rate = to_millimeters(value); // millimeters per minute
        }
        break;
      case 'I': case 'J': case 'K': gc.arc_offset[letter-'I'] = to_millimeters(value); break;
      case 'L': l = trunc(value); break;
      case 'P': p = value; break;                    
      case 'R': gc.arc_radius = to_millimeters(value); break;
      case 'S': 
        if (value < 0) { FAIL(STATUS_INVALID_STATEMENT); } // Cannot be negative
        gc.spindle_speed = value;
        break;
      case 'T': 
        if (value < 0) { FAIL(STATUS_INVALID_STATEMENT); } // Cannot be negative
        gc.tool = trunc(value); 
        break;
      case 'X': target[X_AXIS] = to_millimeters(value); bit_true(axis_words,bit(X_AXIS)); break;
      case 'Y': target[Y_AXIS] = to_millimeters(value); bit_true(axis_words,bit(Y_AXIS)); break;
      case 'Z': target[Z_AXIS] = to_millimeters(value); bit_true(axis_words,bit(Z_AXIS)); break;
      default: FAIL(STATUS_UNSUPPORTED_STATEMENT);
    }
  }
  
  // If there were any errors parsing this line, we will return right away with the bad news
  if (gc.status_code) { return(gc.status_code); }
  
  // Initialize axis index
  uint8_t idx;

  /* Execute Commands: Perform by order of execution defined in NIST RS274-NGC.v3, Table 8, pg.41. */
    
  // ([F]: Set feed rate.)
    
  if (sys.state != STATE_CHECK_MODE) { 
    //  ([M6]: Tool change should be executed here.)

    // [M3,M4,M5]: Update spindle state
    if (bit_istrue(modal_group_words,bit(MODAL_GROUP_7))) {
      spindle_run(gc.spindle_direction, gc.spindle_speed); 
    }
  
    // [*M7,M8,M9]: Update coolant state
    if (bit_istrue(modal_group_words,bit(MODAL_GROUP_8))) {
      coolant_run(gc.coolant_mode);
    }
  }
  
  // [G54,G55,...,G59]: Coordinate system selection
  if ( bit_istrue(modal_group_words,bit(MODAL_GROUP_12)) ) { // Check if called in block
    float coord_data[N_AXIS];
    if (!(settings_read_coord_data(gc.coord_select,coord_data))) { return(STATUS_SETTING_READ_FAIL); } 
    memcpy(gc.coord_system,coord_data,sizeof(coord_data));
  }
  
  // [G4,G10,G28,G30,G92,G92.1]: Perform dwell, set coordinate system data, homing, or set axis offsets.
  // NOTE: These commands are in the same modal group, hence are mutually exclusive. G53 is in this
  // modal group and do not effect these actions.
  switch (non_modal_action) {
    case NON_MODAL_DWELL:
      if (p < 0) { // Time cannot be negative.
        FAIL(STATUS_INVALID_STATEMENT); 
      } else {
        // Ignore dwell in check gcode modes
        if (sys.state != STATE_CHECK_MODE) { mc_dwell(p); }
      }
      break;
    case NON_MODAL_SET_COORDINATE_DATA:
      int_value = trunc(p); // Convert p value to int.
      if ((l != 2 && l != 20) || (int_value < 0 || int_value > N_COORDINATE_SYSTEM)) { // L2 and L20. P1=G54, P2=G55, ... 
        FAIL(STATUS_UNSUPPORTED_STATEMENT); 
      } else if (!axis_words && l==2) { // No axis words.
        FAIL(STATUS_INVALID_STATEMENT);
      } else {
        if (int_value > 0) { int_value--; } // Adjust P1-P6 index to EEPROM coordinate data indexing.
        else { int_value = gc.coord_select; } // Index P0 as the active coordinate system
        float coord_data[N_AXIS];
        if (!settings_read_coord_data(int_value,coord_data)) { return(STATUS_SETTING_READ_FAIL); }
        // Update axes defined only in block. Always in machine coordinates. Can change non-active system.
        for (idx=0; idx<N_AXIS; idx++) { // Axes indices are consistent, so loop may be used.
          if (bit_istrue(axis_words,bit(idx)) ) {
            if (l == 20) {
              coord_data[idx] = gc.position[idx]-target[idx]; // L20: Update axis current position to target
            } else {
              coord_data[idx] = target[idx]; // L2: Update coordinate system axis
            }
          }
        }
        settings_write_coord_data(int_value,coord_data);
        // Update system coordinate system if currently active.
        if (gc.coord_select == int_value) { memcpy(gc.coord_system,coord_data,sizeof(coord_data)); }
      }
      axis_words = 0; // Axis words used. Lock out from motion modes by clearing flags.
      break;
    case NON_MODAL_GO_HOME_0: case NON_MODAL_GO_HOME_1: 
      // Move to intermediate position before going home. Obeys current coordinate system and offsets 
      // and absolute and incremental modes.
      if (axis_words) {
        // Apply absolute mode coordinate offsets or incremental mode offsets.
        for (idx=0; idx<N_AXIS; idx++) { // Axes indices are consistent, so loop may be used.
          if ( bit_istrue(axis_words,bit(idx)) ) {
            if (gc.absolute_mode) {
              target[idx] += gc.coord_system[idx] + gc.coord_offset[idx];
            } else {
              target[idx] += gc.position[idx];
            }
          } else {
            target[idx] = gc.position[idx];
          }
        }
        mc_line(target, -1.0, false);
      }
      // Retreive G28/30 go-home position data (in machine coordinates) from EEPROM
      float coord_data[N_AXIS];
      if (non_modal_action == NON_MODAL_GO_HOME_0) {
        if (!settings_read_coord_data(SETTING_INDEX_G28,coord_data)) { return(STATUS_SETTING_READ_FAIL); }
      } else {
        if (!settings_read_coord_data(SETTING_INDEX_G30,coord_data)) { return(STATUS_SETTING_READ_FAIL); }
      }
      mc_line(coord_data, -1.0, false); 
      memcpy(gc.position, coord_data, sizeof(coord_data)); // gc.position[] = coord_data[];
      axis_words = 0; // Axis words used. Lock out from motion modes by clearing flags.
      break;
    case NON_MODAL_SET_HOME_0: case NON_MODAL_SET_HOME_1:
      if (non_modal_action == NON_MODAL_SET_HOME_0) {
        settings_write_coord_data(SETTING_INDEX_G28,gc.position);
      } else {
        settings_write_coord_data(SETTING_INDEX_G30,gc.position);
      }
      break;
    case NON_MODAL_SET_COORDINATE_OFFSET:
      if (!axis_words) { // No axis words
        FAIL(STATUS_INVALID_STATEMENT);
      } else {
        // Update axes defined only in block. Offsets current system to defined value. Does not update when
        // active coordinate system is selected, but is still active unless G92.1 disables it. 
        for (idx=0; idx<N_AXIS; idx++) { // Axes indices are consistent, so loop may be used.
          if (bit_istrue(axis_words,bit(idx)) ) {
            gc.coord_offset[idx] = gc.position[idx]-gc.coord_system[idx]-target[idx];
          }
        }
      }
      axis_words = 0; // Axis words used. Lock out from motion modes by clearing flags.
      break;
    case NON_MODAL_RESET_COORDINATE_OFFSET: 
      clear_vector(gc.coord_offset); // Disable G92 offsets by zeroing offset vector.
      break;
  }

  // [G0,G1,G2,G3,G80]: Perform motion modes. 
  // NOTE: Commands G10,G28,G30,G92 lock out and prevent axis words from use in motion modes. 
  // Enter motion modes only if there are axis words or a motion mode command word in the block.
  if ( bit_istrue(modal_group_words,bit(MODAL_GROUP_1)) || axis_words ) {

    // G1,G2,G3 require F word in inverse time mode.  
    if ( gc.inverse_feed_rate_mode ) { 
      if (inverse_feed_rate < 0 && gc.motion_mode != MOTION_MODE_CANCEL) {
        FAIL(STATUS_INVALID_STATEMENT);
      }
    }
    // Absolute override G53 only valid with G0 and G1 active.
    if ( absolute_override && !(gc.motion_mode == MOTION_MODE_SEEK || gc.motion_mode == MOTION_MODE_LINEAR)) {
      FAIL(STATUS_INVALID_STATEMENT);
    }
    // Report any errors.  
    if (gc.status_code) { return(gc.status_code); }

    // Convert all target position data to machine coordinates for executing motion. Apply
    // absolute mode coordinate offsets or incremental mode offsets.
    // NOTE: Tool offsets may be appended to these conversions when/if this feature is added.
    for (idx=0; idx<N_AXIS; idx++) { // Axes indices are consistent, so loop may be used to save flash space.
      if ( bit_istrue(axis_words,bit(idx)) ) {
        if (!absolute_override) { // Do not update target in absolute override mode
          if (gc.absolute_mode) {
            target[idx] += gc.coord_system[idx] + gc.coord_offset[idx]; // Absolute mode
          } else {
            target[idx] += gc.position[idx]; // Incremental mode
          }
        }
      } else {
        target[idx] = gc.position[idx]; // No axis word in block. Keep same axis position.
      }
    }
  
    switch (gc.motion_mode) {
      case MOTION_MODE_CANCEL: 
        if (axis_words) { FAIL(STATUS_INVALID_STATEMENT); } // No axis words allowed while active.
        break;
      case MOTION_MODE_SEEK:
        if (!axis_words) { FAIL(STATUS_INVALID_STATEMENT);} 
        else { mc_line(target, -1.0, false); }
        break;
      case MOTION_MODE_LINEAR:
        // TODO: Inverse time requires F-word with each statement. Need to do a check. Also need
        // to check for initial F-word upon startup. Maybe just set to zero upon initialization
        // and after an inverse time move and then check for non-zero feed rate each time. This
        // should be efficient and effective.
        if (!axis_words) { FAIL(STATUS_INVALID_STATEMENT);} 
        else { mc_line(target, (gc.inverse_feed_rate_mode) ? inverse_feed_rate : gc.feed_rate, gc.inverse_feed_rate_mode); }
        break;
      case MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC:
        // Check if at least one of the axes of the selected plane has been specified. If in center 
        // format arc mode, also check for at least one of the IJK axes of the selected plane was sent.
        if ( !( bit_false(axis_words,bit(gc.plane_axis_2)) ) || 
             ( !gc.arc_radius && !gc.arc_offset[gc.plane_axis_0] && !gc.arc_offset[gc.plane_axis_1] ) ) { 
          FAIL(STATUS_INVALID_STATEMENT);
        } else {
          if (gc.arc_radius != 0) { // Arc Radius Mode
            // Compute arc radius and offsets
            gc_convert_arc_radius_mode(target);
            if (gc.status_code) { return(gc.status_code); }
          } else { // Arc Center Format Offset Mode            
            gc.arc_radius = hypot(gc.arc_offset[gc.plane_axis_0], gc.arc_offset[gc.plane_axis_1]); // Compute arc radius for mc_arc
          }
          
          // Set clockwise/counter-clockwise sign for mc_arc computations
          uint8_t isclockwise = false;
          if (gc.motion_mode == MOTION_MODE_CW_ARC) { isclockwise = true; }
    
          // Trace the arc
          mc_arc(gc.position, target, gc.arc_offset, gc.plane_axis_0, gc.plane_axis_1, gc.plane_axis_2,
            (gc.inverse_feed_rate_mode) ? inverse_feed_rate : gc.feed_rate, gc.inverse_feed_rate_mode,
            gc.arc_radius, isclockwise);
        }            
        break;
    }
    
    // Report any errors.
    if (gc.status_code) { return(gc.status_code); }    
    
    // As far as the parser is concerned, the position is now == target. In reality the
    // motion control system might still be processing the action and the real tool position
    // in any intermediate location.
    memcpy(gc.position, target, sizeof(target)); // gc.position[] = target[];
  }
  
  // M0,M1,M2,M30: Perform non-running program flow actions. During a program pause, the buffer may 
  // refill and can only be resumed by the cycle start run-time command.
  if (gc.program_flow) { 
    protocol_buffer_synchronize(); // Finish all remaining buffered motions. Program paused when complete.
    sys.auto_start = false; // Disable auto cycle start. Forces pause until cycle start issued.
  
    // If complete, reset to reload defaults (G92.2,G54,G17,G90,G94,M48,G40,M5,M9). Otherwise,
    // re-enable program flow after pause complete, where cycle start will resume the program.
    if (gc.program_flow == PROGRAM_FLOW_COMPLETED) { mc_reset(); }
    else { gc.program_flow = PROGRAM_FLOW_RUNNING; }
  }
  
  return(gc.status_code);
}

// Parses the next statement and leaves the counter on the first character following
// the statement. Returns 1 if there was a statements, 0 if end of string was reached
// or there was an error (check state.status_code).
static uint8_t next_statement(char *letter, float *float_ptr, char *line, uint8_t *char_counter) 
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
  if (!read_float(line, char_counter, float_ptr)) {
    FAIL(STATUS_BAD_NUMBER_FORMAT); 
    return(0);
  };
  return(1);
}


static void gc_convert_arc_radius_mode(float *target)  
{
/*  We need to calculate the center of the circle that has the designated radius and passes
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
    j = (y + (x * h_x2_div_d))/2        */
  
  // Calculate the change in position along each selected axis
  float x = target[gc.plane_axis_0]-gc.position[gc.plane_axis_0];
  float y = target[gc.plane_axis_1]-gc.position[gc.plane_axis_1];
  
  clear_vector(gc.arc_offset);
  // First, use h_x2_div_d to compute 4*h^2 to check if it is negative or r is smaller
  // than d. If so, the sqrt of a negative number is complex and error out.
  float h_x2_div_d = 4 * gc.arc_radius*gc.arc_radius - x*x - y*y;
  if (h_x2_div_d < 0) { FAIL(STATUS_ARC_RADIUS_ERROR); return; }
  // Finish computing h_x2_div_d.
  h_x2_div_d = -sqrt(h_x2_div_d)/hypot(x,y); // == -(h * 2 / d)
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
  if (gc.arc_radius < 0) { 
      h_x2_div_d = -h_x2_div_d; 
      gc.arc_radius = -gc.arc_radius; // Finished with r. Set to positive for mc_arc
  }        
  // Complete the operation by calculating the actual center of the arc
  gc.arc_offset[gc.plane_axis_0] = 0.5*(x-(y*h_x2_div_d));
  gc.arc_offset[gc.plane_axis_1] = 0.5*(y+(x*h_x2_div_d));
}   

/* 
  Not supported:

  - Canned cycles
  - Tool radius compensation
  - A,B,C-axes
  - Evaluation of expressions
  - Variables
  - Probing
  - Override control (TBD)
  - Tool changes
  - Switches
   
   (*) Indicates optional parameter, enabled through config.h and re-compile
   group 0 = {G92.2, G92.3} (Non modal: Cancel and re-enable G92 offsets)
   group 1 = {G38.2, G81 - G89} (Motion modes: straight probe, canned cycles)
   group 4 = {M1} (Optional stop, ignored)
   group 6 = {M6} (Tool change)
   group 8 = {*M7} enable mist coolant
   group 9 = {M48, M49} enable/disable feed and speed override switches
   group 13 = {G61, G61.1, G64} path control mode
*/
