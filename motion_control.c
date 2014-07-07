/*
  motion_control.c - high level interface for issuing motion commands
  Part of Grbl

  Copyright (c) 2011-2014 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
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

#include "system.h"
#include "settings.h"
#include "protocol.h"
#include "gcode.h"
#include "planner.h"
#include "stepper.h"
#include "motion_control.h"
#include "spindle_control.h"
#include "coolant_control.h"
#include "limits.h"
#include "probe.h"
#include "report.h"


// Execute linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
// unless invert_feed_rate is true. Then the feed_rate means that the motion should be completed in
// (1 minute)/feed_rate time.
// NOTE: This is the primary gateway to the grbl planner. All line motions, including arc line 
// segments, must pass through this routine before being passed to the planner. The seperation of
// mc_line and plan_buffer_line is done primarily to place non-planner-type functions from being
// in the planner and to let backlash compensation or canned cycle integration simple and direct.
#ifdef USE_LINE_NUMBERS
void mc_line(float *target, float feed_rate, uint8_t invert_feed_rate, int32_t line_number)
#else
void mc_line(float *target, float feed_rate, uint8_t invert_feed_rate)
#endif
{
  // If enabled, check for soft limit violations. Placed here all line motions are picked up
  // from everywhere in Grbl.
  if (bit_istrue(settings.flags,BITFLAG_SOFT_LIMIT_ENABLE)) { limits_soft_check(target); }    
      
  // If in check gcode mode, prevent motion by blocking planner. Soft limits still work.
  if (sys.state == STATE_CHECK_MODE) { return; }
    
  // NOTE: Backlash compensation may be installed here. It will need direction info to track when
  // to insert a backlash line motion(s) before the intended line motion and will require its own
  // plan_check_full_buffer() and check for system abort loop. Also for position reporting 
  // backlash steps will need to be also tracked, which will need to be kept at a system level.
  // There are likely some other things that will need to be tracked as well. However, we feel
  // that backlash compensation should NOT be handled by Grbl itself, because there are a myriad
  // of ways to implement it and can be effective or ineffective for different CNC machines. This
  // would be better handled by the interface as a post-processor task, where the original g-code
  // is translated and inserts backlash motions that best suits the machine. 
  // NOTE: Perhaps as a middle-ground, all that needs to be sent is a flag or special command that
  // indicates to Grbl what is a backlash compensation motion, so that Grbl executes the move but
  // doesn't update the machine position values. Since the position values used by the g-code
  // parser and planner are separate from the system machine positions, this is doable.

  // If the buffer is full: good! That means we are well ahead of the robot. 
  // Remain in this loop until there is room in the buffer.
  do {
    protocol_execute_runtime(); // Check for any run-time commands
    if (sys.abort) { return; } // Bail, if system abort.
    if ( plan_check_full_buffer() ) { protocol_auto_cycle_start(); } // Auto-cycle start when buffer is full.
    else { break; }
  } while (1);

  #ifdef USE_LINE_NUMBERS
  plan_buffer_line(target, feed_rate, invert_feed_rate, line_number);
  #else
  plan_buffer_line(target, feed_rate, invert_feed_rate);
  #endif
  
  // If idle, indicate to the system there is now a planned block in the buffer ready to cycle 
  // start. Otherwise ignore and continue on.
  if (!sys.state) { sys.state = STATE_QUEUED; }
}


// Execute an arc in offset mode format. position == current xyz, target == target xyz, 
// offset == offset from current xyz, axis_XXX defines circle plane in tool space, axis_linear is
// the direction of helical travel, radius == circle radius, isclockwise boolean. Used
// for vector transformation direction.
// The arc is approximated by generating a huge number of tiny, linear segments. The chordal tolerance
// of each segment is configured in settings.arc_tolerance, which is defined to be the maximum normal
// distance from segment to the circle when the end points both lie on the circle.
#ifdef USE_LINE_NUMBERS
void mc_arc(float *position, float *target, float *offset, float radius, float feed_rate, 
  uint8_t invert_feed_rate, uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, int32_t line_number)
#else
void mc_arc(float *position, float *target, float *offset, float radius, float feed_rate,
  uint8_t invert_feed_rate, uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear)
#endif
{
  float center_axis0 = position[axis_0] + offset[axis_0];
  float center_axis1 = position[axis_1] + offset[axis_1];
  float r_axis0 = -offset[axis_0];  // Radius vector from center to current location
  float r_axis1 = -offset[axis_1];
  float rt_axis0 = target[axis_0] - center_axis0;
  float rt_axis1 = target[axis_1] - center_axis1;
  
  // CCW angle between position and target from circle center. Only one atan2() trig computation required.
  float angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);
  if (gc_state.modal.motion == MOTION_MODE_CW_ARC) { // Correct atan2 output per direction
    if (angular_travel >= 0) { angular_travel -= 2*M_PI; }
  } else {
    if (angular_travel <= 0) { angular_travel += 2*M_PI; }
  }

  // NOTE: Segment end points are on the arc, which can lead to the arc diameter being smaller by up to
  // (2x) settings.arc_tolerance. For 99% of users, this is just fine. If a different arc segment fit
  // is desired, i.e. least-squares, midpoint on arc, just change the mm_per_arc_segment calculation.
  // For the intended uses of Grbl, this value shouldn't exceed 2000 for the strictest of cases.
  uint16_t segments = floor(fabs(0.5*angular_travel*radius)/
                          sqrt(settings.arc_tolerance*(2*radius - settings.arc_tolerance)) );
  
  if (segments) { 
    // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
    // by a number of discrete segments. The inverse feed_rate should be correct for the sum of 
    // all segments.
    if (invert_feed_rate) { feed_rate *= segments; }
   
    float theta_per_segment = angular_travel/segments;
    float linear_per_segment = (target[axis_linear] - position[axis_linear])/segments;

    /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
       and phi is the angle of rotation. Solution approach by Jens Geisler.
           r_T = [cos(phi) -sin(phi);
                  sin(phi)  cos(phi] * r ;
       
       For arc generation, the center of the circle is the axis of rotation and the radius vector is 
       defined from the circle center to the initial position. Each line segment is formed by successive
       vector rotations. Single precision values can accumulate error greater than tool precision in some
       cases. So, exact arc path correction is implemented. This approach avoids the problem of too many very
       expensive trig operations [sin(),cos(),tan()] which can take 100-200 usec each to compute.
  
       Small angle approximation may be used to reduce computation overhead further. A third-order approximation
       (second order sin() has too much error) holds for nearly all CNC applications, except for possibly very
       small radii (~0.5mm). In other words, theta_per_segment would need to be greater than 0.25 rad(14 deg) 
       and N_ARC_CORRECTION would need to be large to cause an appreciable drift error (>5% of radius, for very
       small radii, 5% of 0.5mm is very, very small). N_ARC_CORRECTION~=20 should be more than small enough to 
       correct for numerical drift error. Also decreasing the tolerance will improve the approximation too.
       
       This approximation also allows mc_arc to immediately insert a line segment into the planner 
       without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
       a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead. 
       This is important when there are successive arc motions. 
    */
    // Computes: cos_T = 1 - theta_per_segment^2/2, sin_T = theta_per_segment - theta_per_segment^3/6) in ~52usec
    float cos_T = 2.0 - theta_per_segment*theta_per_segment;
    float sin_T = theta_per_segment*0.16666667*(cos_T + 4.0);
    cos_T *= 0.5;

    float sin_Ti;
    float cos_Ti;
    float r_axisi;
    uint16_t i;
    uint8_t count = 0;
  
    for (i = 1; i<segments; i++) { // Increment (segments-1).
      
      if (count < N_ARC_CORRECTION) {
        // Apply vector rotation matrix. ~40 usec
        r_axisi = r_axis0*sin_T + r_axis1*cos_T;
        r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
        r_axis1 = r_axisi;
        count++;
      } else {      
        // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments. ~375 usec
        // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
        cos_Ti = cos(i*theta_per_segment);
        sin_Ti = sin(i*theta_per_segment);
        r_axis0 = -offset[axis_0]*cos_Ti + offset[axis_1]*sin_Ti;
        r_axis1 = -offset[axis_0]*sin_Ti - offset[axis_1]*cos_Ti;
        count = 0;
      }
  
      // Update arc_target location
      position[axis_0] = center_axis0 + r_axis0;
      position[axis_1] = center_axis1 + r_axis1;
      position[axis_linear] += linear_per_segment;
      
      #ifdef USE_LINE_NUMBERS
      mc_line(position, feed_rate, invert_feed_rate, line_number);
      #else
      mc_line(position, feed_rate, invert_feed_rate);
      #endif
      
      // Bail mid-circle on system abort. Runtime command check already performed by mc_line.
      if (sys.abort) { return; }
    }
  }
  // Ensure last segment arrives at target location.
  #ifdef USE_LINE_NUMBERS
  mc_line(target, feed_rate, invert_feed_rate, line_number);
  #else
  mc_line(target, feed_rate, invert_feed_rate);
  #endif
}


// Execute dwell in seconds.
void mc_dwell(float seconds) 
{
   if (sys.state == STATE_CHECK_MODE) { return; }
   
   uint16_t i = floor(1000/DWELL_TIME_STEP*seconds);
   protocol_buffer_synchronize();
   delay_ms(floor(1000*seconds-i*DWELL_TIME_STEP)); // Delay millisecond remainder.
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
void mc_homing_cycle()
{
  sys.state = STATE_HOMING; // Set system state variable
  limits_disable(); // Disable hard limits pin change register for cycle duration
    
  // -------------------------------------------------------------------------------------
  // Perform homing routine. NOTE: Special motion case. Only system reset works.
  
  // Search to engage all axes limit switches at faster homing seek rate.
  limits_go_home(HOMING_CYCLE_0);  // Homing cycle 0
  #ifdef HOMING_CYCLE_1
    limits_go_home(HOMING_CYCLE_1);  // Homing cycle 1
  #endif
  #ifdef HOMING_CYCLE_2
    limits_go_home(HOMING_CYCLE_2);  // Homing cycle 2
  #endif
    
  protocol_execute_runtime(); // Check for reset and set system abort.
  if (sys.abort) { return; } // Did not complete. Alarm state set by mc_alarm.

  // Homing cycle complete! Setup system for normal operation.
  // -------------------------------------------------------------------------------------

  // Gcode parser position was circumvented by the limits_go_home() routine, so sync position now.
  gc_sync_position();
  
  // Set idle state after homing completes and before returning to main program.  
  sys.state = STATE_IDLE;
  st_go_idle(); // Set idle state after homing completes

  // If hard limits feature enabled, re-enable hard limits pin change register after homing cycle.
  limits_init();
}


// Perform tool length probe cycle. Requires probe switch.
// NOTE: Upon probe failure, the program will be stopped and placed into ALARM state.
#ifdef USE_LINE_NUMBERS
void mc_probe_cycle(float *target, float feed_rate, uint8_t invert_feed_rate, int32_t line_number)
#else
void mc_probe_cycle(float *target, float feed_rate, uint8_t invert_feed_rate)
#endif
{
  if (sys.state != STATE_CYCLE) protocol_auto_cycle_start();
  protocol_buffer_synchronize(); // Finish all queued commands
  if (sys.abort) { return; } // Return if system reset has been issued.

  // Perform probing cycle. Planner buffer should be empty at this point.
  #ifdef USE_LINE_NUMBERS
  mc_line(target, feed_rate, invert_feed_rate, line_number);
  #else
  mc_line(target, feed_rate, invert_feed_rate);
  #endif

  // NOTE: Parser error-checking ensures the probe isn't already closed/triggered.
  sys.probe_state = PROBE_ACTIVE;

  bit_true_atomic(sys.execute, EXEC_CYCLE_START);
  do {
    protocol_execute_runtime(); 
    if (sys.abort) { return; } // Check for system abort
  } while ((sys.state != STATE_IDLE) && (sys.state != STATE_QUEUED));

  if (sys.probe_state == PROBE_ACTIVE) { bit_true_atomic(sys.execute, EXEC_CRIT_EVENT); }
  protocol_execute_runtime();   // Check and execute run-time commands
  if (sys.abort) { return; } // Check for system abort

  //Prep the new target based on the position that the probe triggered
  uint8_t i;
  for(i=0; i<N_AXIS; ++i){
    target[i] = (float)sys.probe_position[i]/settings.steps_per_mm[i];
  }

  protocol_execute_runtime();

  st_reset(); // Immediately force kill steppers and reset step segment buffer.
  plan_reset(); // Reset planner buffer. Zero planner positions. Ensure homing motion is cleared.
  plan_sync_position(); // Sync planner position to current machine position for pull-off move.

  #ifdef USE_LINE_NUMBERS
  mc_line(target, feed_rate, invert_feed_rate, line_number); // Bypass mc_line(). Directly plan homing motion.
  #else
  mc_line(target, feed_rate, invert_feed_rate); // Bypass mc_line(). Directly plan homing motion.
  #endif

  bit_true_atomic(sys.execute, EXEC_CYCLE_START);
  protocol_buffer_synchronize(); // Complete pull-off motion.
  if (sys.abort) { return; } // Did not complete. Alarm state set by mc_alarm.

  // Gcode parser position was circumvented by the this routine, so sync position now.
  gc_sync_position();

  // Output the probe position as message.
  report_probe_parameters();
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
    bit_true_atomic(sys.execute, EXEC_RESET);

    // Kill spindle and coolant.   
    spindle_stop();
    coolant_stop();

    // Kill steppers only if in any motion state, i.e. cycle, feed hold, homing, or jogging
    // NOTE: If steppers are kept enabled via the step idle delay setting, this also keeps
    // the steppers enabled by avoiding the go_idle call altogether, unless the motion state is
    // violated, by which, all bets are off.
    if (sys.state & (STATE_CYCLE | STATE_HOLD | STATE_HOMING)) {
      bit_true_atomic(sys.execute, EXEC_ALARM); // Flag main program to execute alarm state.
      st_go_idle(); // Force kill steppers. Position has likely been lost.
    }
  }
}
