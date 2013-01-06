/*
  motion_control.c - high level interface for issuing motion commands
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011-2012 Sungeun K. Jeon
  Copyright (c) 2011 Jens Geisler
  
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
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#include "settings.h"
#include "config.h"
#include "gcode.h"
#include "motion_control.h"
#include "spindle_control.h"
#include "coolant_control.h"
#include "nuts_bolts.h"
#include "stepper.h"
#include "planner.h"
#include "limits.h"
#include "protocol.h"

// Execute linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
// unless invert_feed_rate is true. Then the feed_rate means that the motion should be completed in
// (1 minute)/feed_rate time.
// NOTE: This is the primary gateway to the grbl planner. All line motions, including arc line 
// segments, must pass through this routine before being passed to the planner. The seperation of
// mc_line and plan_buffer_line is done primarily to make backlash compensation integration simple
// and direct.
// TODO: Check for a better way to avoid having to push the arguments twice for non-backlash cases.
// However, this keeps the memory requirements lower since it doesn't have to call and hold two 
// plan_buffer_lines in memory. Grbl only has to retain the original line input variables during a
// backlash segment(s).
void mc_line(float x, float y, float z, float feed_rate, uint8_t invert_feed_rate)
{
  // TODO: Perform soft limit check here. Just check if the target x,y,z values are outside the 
  // work envelope. Should be straightforward and efficient. By placing it here, rather than in 
  // the g-code parser, it directly picks up motions from everywhere in Grbl.

  // If in check gcode mode, prevent motion by blocking planner.
  if (sys.state == STATE_CHECK_MODE) { return; }
    
  // TODO: Backlash compensation may be installed here. Only need direction info to track when
  // to insert a backlash line motion(s) before the intended line motion. Requires its own
  // plan_check_full_buffer() and check for system abort loop. Also for position reporting 
  // backlash steps will need to be also tracked. Not sure what the best strategy is for this,
  // i.e. keep the planner independent and do the computations in the status reporting, or let
  // the planner handle the position corrections. The latter may get complicated.

  // If the buffer is full: good! That means we are well ahead of the robot. 
  // Remain in this loop until there is room in the buffer.
  do {
    protocol_execute_runtime(); // Check for any run-time commands
    if (sys.abort) { return; } // Bail, if system abort.
  } while ( plan_check_full_buffer() );
  plan_buffer_line(x, y, z, feed_rate, invert_feed_rate);
  
  // If idle, indicate to the system there is now a planned block in the buffer ready to cycle 
  // start. Otherwise ignore and continue on.
  if (!sys.state) { sys.state = STATE_QUEUED; }
  
  // Auto-cycle start immediately after planner finishes. Enabled/disabled by grbl settings. During 
  // a feed hold, auto-start is disabled momentarily until the cycle is resumed by the cycle-start 
  // runtime command.
  // NOTE: This is allows the user to decide to exclusively use the cycle start runtime command to
  // begin motion or let grbl auto-start it for them. This is useful when: manually cycle-starting
  // when the buffer is completely full and primed; auto-starting, if there was only one g-code 
  // command sent during manual operation; or if a system is prone to buffer starvation, auto-start
  // helps make sure it minimizes any dwelling/motion hiccups and keeps the cycle going. 
  if (sys.auto_start) { st_cycle_start(); }
}


// Execute an arc in offset mode format. position == current xyz, target == target xyz, 
// offset == offset from current xyz, axis_XXX defines circle plane in tool space, axis_linear is
// the direction of helical travel, radius == circle radius, isclockwise boolean. Used
// for vector transformation direction.
// The arc is approximated by generating a huge number of tiny, linear segments. The length of each 
// segment is configured in settings.mm_per_arc_segment.  
void mc_arc(float *position, float *target, float *offset, uint8_t axis_0, uint8_t axis_1, 
  uint8_t axis_linear, float feed_rate, uint8_t invert_feed_rate, float radius, uint8_t isclockwise)
{      
  float center_axis0 = position[axis_0] + offset[axis_0];
  float center_axis1 = position[axis_1] + offset[axis_1];
  float linear_travel = target[axis_linear] - position[axis_linear];
  float r_axis0 = -offset[axis_0];  // Radius vector from center to current location
  float r_axis1 = -offset[axis_1];
  float rt_axis0 = target[axis_0] - center_axis0;
  float rt_axis1 = target[axis_1] - center_axis1;
  
  // CCW angle between position and target from circle center. Only one atan2() trig computation required.
  float angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);
  if (isclockwise) { // Correct atan2 output per direction
    if (angular_travel >= 0) { angular_travel -= 2*M_PI; }
  } else {
    if (angular_travel <= 0) { angular_travel += 2*M_PI; }
  }
  
  float millimeters_of_travel = hypot(angular_travel*radius, fabs(linear_travel));
  if (millimeters_of_travel == 0.0) { return; }
  uint16_t segments = floor(millimeters_of_travel/settings.mm_per_arc_segment);
  // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
  // by a number of discrete segments. The inverse feed_rate should be correct for the sum of 
  // all segments.
  if (invert_feed_rate) { feed_rate *= segments; }
 
  float theta_per_segment = angular_travel/segments;
  float linear_per_segment = linear_travel/segments;
  
  /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
     and phi is the angle of rotation. Solution approach by Jens Geisler.
         r_T = [cos(phi) -sin(phi);
                sin(phi)  cos(phi] * r ;
     
     For arc generation, the center of the circle is the axis of rotation and the radius vector is 
     defined from the circle center to the initial position. Each line segment is formed by successive
     vector rotations. This requires only two cos() and sin() computations to form the rotation
     matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
     all double numbers are single precision on the Arduino. (True double precision will not have
     round off issues for CNC applications.) Single precision error can accumulate to be greater than
     tool precision in some cases. Therefore, arc path correction is implemented. 

     Small angle approximation may be used to reduce computation overhead further. This approximation
     holds for everything, but very small circles and large mm_per_arc_segment values. In other words,
     theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
     to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for 
     numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
     issue for CNC machines with the single precision Arduino calculations.
     
     This approximation also allows mc_arc to immediately insert a line segment into the planner 
     without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
     a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead. 
     This is important when there are successive arc motions. 
  */
  // Vector rotation matrix values
  float cos_T = 1-0.5*theta_per_segment*theta_per_segment; // Small angle approximation
  float sin_T = theta_per_segment;
  
  float arc_target[3];
  float sin_Ti;
  float cos_Ti;
  float r_axisi;
  uint16_t i;
  int8_t count = 0;

  // Initialize the linear axis
  arc_target[axis_linear] = position[axis_linear];

  for (i = 1; i<segments; i++) { // Increment (segments-1)
    
    if (count < settings.n_arc_correction) {
      // Apply vector rotation matrix 
      r_axisi = r_axis0*sin_T + r_axis1*cos_T;
      r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
      r_axis1 = r_axisi;
      count++;
    } else {
      // Arc correction to radius vector. Computed only every n_arc_correction increments.
      // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
      cos_Ti = cos(i*theta_per_segment);
      sin_Ti = sin(i*theta_per_segment);
      r_axis0 = -offset[axis_0]*cos_Ti + offset[axis_1]*sin_Ti;
      r_axis1 = -offset[axis_0]*sin_Ti - offset[axis_1]*cos_Ti;
      count = 0;
    }

    // Update arc_target location
    arc_target[axis_0] = center_axis0 + r_axis0;
    arc_target[axis_1] = center_axis1 + r_axis1;
    arc_target[axis_linear] += linear_per_segment;
    mc_line(arc_target[X_AXIS], arc_target[Y_AXIS], arc_target[Z_AXIS], feed_rate, invert_feed_rate);
    
    // Bail mid-circle on system abort. Runtime command check already performed by mc_line.
    if (sys.abort) { return; }
  }
  // Ensure last segment arrives at target location.
  mc_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], feed_rate, invert_feed_rate);
}


// Execute dwell in seconds.
void mc_dwell(float seconds) 
{
   uint16_t i = floor(1000/DWELL_TIME_STEP*seconds);
   plan_synchronize();
   delay_ms(floor(1000*seconds-i*DWELL_TIME_STEP)); // Delay millisecond remainder
   while (i-- > 0) {
     // NOTE: Check and execute runtime commands during dwell every <= DWELL_TIME_STEP milliseconds.
     protocol_execute_runtime();
     if (sys.abort) { return; }
     _delay_ms(DWELL_TIME_STEP); // Delay DWELL_TIME_STEP increment
   }
}


// Perform homing cycle to locate and set machine zero. Only '$H' executes this command.
// NOTE: There should be no motions in the buffer and Grbl must be in an idle state before
// executing the homing cycle. This prevents incorrect buffered plans after homing.
void mc_go_home()
{
  sys.state = STATE_HOMING; // Set system state variable
  LIMIT_PCMSK &= ~LIMIT_MASK; // Disable hard limits pin change register for cycle duration
  
  limits_go_home(); // Perform homing routine.

  protocol_execute_runtime(); // Check for reset and set system abort.
  if (sys.abort) { return; } // Did not complete. Alarm state set by mc_alarm.

  // The machine should now be homed and machine zero has been located. Upon completion, 
  // reset system position and sync internal position vectors.
  clear_vector_float(sys.position); // Set machine zero
  sys_sync_current_position();
  sys.state = STATE_IDLE; // Set system state to IDLE to complete motion and indicate homed.
  
  // Pull-off axes (that have been homed) from limit switches before continuing motion. 
  // This provides some initial clearance off the switches and should also help prevent them 
  // from falsely tripping when hard limits are enabled.
  int8_t x_dir, y_dir, z_dir;
  x_dir = y_dir = z_dir = 0;
  if (HOMING_LOCATE_CYCLE & (1<<X_AXIS)) { 
    if (settings.homing_dir_mask & (1<<X_DIRECTION_BIT)) { x_dir = 1; }
    else { x_dir = -1; }
  }
  if (HOMING_LOCATE_CYCLE & (1<<Y_AXIS)) { 
    if (settings.homing_dir_mask & (1<<Y_DIRECTION_BIT)) { y_dir = 1; }
    else { y_dir = -1; }
  }
  if (HOMING_LOCATE_CYCLE & (1<<Z_AXIS)) { 
    if (settings.homing_dir_mask & (1<<Z_DIRECTION_BIT)) { z_dir = 1; }
    else { z_dir = -1; }
  }
  mc_line(x_dir*settings.homing_pulloff, y_dir*settings.homing_pulloff, 
          z_dir*settings.homing_pulloff, settings.homing_seek_rate, false);
  st_cycle_start(); // Move it. Nothing should be in the buffer except this motion. 
  plan_synchronize(); // Make sure the motion completes.
  
  // The gcode parser position circumvented by the pull-off maneuver, so sync position vectors.
  sys_sync_current_position();

  // If hard limits feature enabled, re-enable hard limits pin change register after homing cycle.
  if (bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE)) { LIMIT_PCMSK |= LIMIT_MASK; }
  // Finished! 
}


// Method to ready the system to reset by setting the runtime reset command and killing any
// active processes in the system. This also checks if a system reset is issued while Grbl
// is in a motion state. If so, kills the steppers and sets the system alarm to flag position
// lost, since there was an abrupt uncontrolled deceleration. Called at an interrupt level by
// runtime abort command and hard limits. So, keep to a minimum.
void mc_reset()
{
  // Only this function can set the system reset. Helps prevent multiple kill calls.
  if (bit_isfalse(sys.execute, EXEC_RESET)) {
    sys.execute |= EXEC_RESET;

    // Kill spindle and coolant.   
    spindle_stop();
    coolant_stop();

    // Kill steppers only if in any motion state, i.e. cycle, feed hold, homing, or jogging
    // NOTE: If steppers are kept enabled via the step idle delay setting, this also keeps
    // the steppers enabled by avoiding the go_idle call altogether, unless the motion state is
    // violated, by which, all bets are off.
    switch (sys.state) {
      case STATE_CYCLE: case STATE_HOLD: case STATE_HOMING: // case STATE_JOG:
        sys.execute |= EXEC_ALARM; // Execute alarm state.
        st_go_idle(); // Execute alarm force kills steppers. Position likely lost.
    }
  }
}
