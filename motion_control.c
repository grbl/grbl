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
#include "settings.h"
#include "config.h"
#include "motion_control.h"
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
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
void mc_line(double x, double y, double z, double feed_rate, uint8_t invert_feed_rate)
{
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
void mc_arc(double *position, double *target, double *offset, uint8_t axis_0, uint8_t axis_1, 
  uint8_t axis_linear, double feed_rate, uint8_t invert_feed_rate, double radius, uint8_t isclockwise)
{      
  double center_axis0 = position[axis_0] + offset[axis_0];
  double center_axis1 = position[axis_1] + offset[axis_1];
  double linear_travel = target[axis_linear] - position[axis_linear];
  double r_axis0 = -offset[axis_0];  // Radius vector from center to current location
  double r_axis1 = -offset[axis_1];
  double rt_axis0 = target[axis_0] - center_axis0;
  double rt_axis1 = target[axis_1] - center_axis1;
  
  // CCW angle between position and target from circle center. Only one atan2() trig computation required.
  double angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);
  if (angular_travel < 0) { angular_travel += 2*M_PI; }
  if (isclockwise) { angular_travel -= 2*M_PI; }
  
  double millimeters_of_travel = hypot(angular_travel*radius, fabs(linear_travel));
  if (millimeters_of_travel == 0.0) { return; }
  uint16_t segments = floor(millimeters_of_travel/settings.mm_per_arc_segment);
  // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
  // by a number of discrete segments. The inverse feed_rate should be correct for the sum of 
  // all segments.
  if (invert_feed_rate) { feed_rate *= segments; }
 
  double theta_per_segment = angular_travel/segments;
  double linear_per_segment = linear_travel/segments;
  
  /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
     and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
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
  double cos_T = 1-0.5*theta_per_segment*theta_per_segment; // Small angle approximation
  double sin_T = theta_per_segment;
  
  double arc_target[3];
  double sin_Ti;
  double cos_Ti;
  double r_axisi;
  uint16_t i;
  int8_t count = 0;

  // Initialize the linear axis
  arc_target[axis_linear] = position[axis_linear];

  for (i = 1; i<segments; i++) { // Increment (segments-1)
    
    if (count < N_ARC_CORRECTION) {
      // Apply vector rotation matrix 
      r_axisi = r_axis0*sin_T + r_axis1*cos_T;
      r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
      r_axis1 = r_axisi;
      count++;
    } else {
      // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
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
void mc_dwell(double seconds) 
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


// TODO: Update limits and homing cycle subprograms for better integration with new features.
void mc_go_home()
{
  limits_go_home();  
  plan_set_current_position(0,0,0);
}
