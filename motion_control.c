/*
  motion_control.c - cartesian robot controller.
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

/* The structure of this module was inspired by the Arduino GCode_Interpreter by Mike Ellery. The arc
   interpolator written from the information provided in the Wikipedia article 'Midpoint circle algorithm'
   and the lecture 'Circle Drawing Algorithms' by Leonard McMillan. 
   
   http://en.wikipedia.org/wiki/Midpoint_circle_algorithm
   http://www.cs.unc.edu/~mcmillan/comp136/Lecture7/circle.html
*/

#include <avr/io.h>
#include "config.h"
#include "motion_control.h"
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#include "nuts_bolts.h"
#include "stepper.h"
#include "serial_protocol.h"

#include "wiring_serial.h"

#define ONE_MINUTE_OF_MICROSECONDS 60000000.0

// Parameters when mode is MC_MODE_ARC
struct LinearMotionParameters {
  int8_t direction[3];  // The direction of travel along each axis (-1, 0 or 1)
  uint16_t feed_rate;
  int32_t target[3], // The target position in absolute steps
    step_count[3],   // Absolute steps of travel along each axis
    counter[3],      // A counter used in the bresenham algorithm for line plotting
    maximum_steps;   // The larges absolute step-count of any axis
};

struct ArcMotionParameters {
  int8_t direction[3];                             // The direction of travel along each axis (-1, 0 or 1)
  int8_t angular_direction;                        // 1 = clockwise, -1 = anticlockwise
  int32_t x, y, target_x, target_y;                // current position and target position in the 
                                                   // local coordinate system of the arc-generator where [0,0] is the 
                                                   // center of the arc.
  int target_direction_x, target_direction_y;      // signof(target_x)*angular_direction precalculated for speed                                                 
  int32_t error, x2, y2;                           // error is always == (x**2 + y**2 - radius**2), 
                                                   // x2 is always 2*x, y2 is always 2*y
  uint8_t axis_x, axis_y;                          // maps the arc axes to stepper axes
  int32_t target[3];                               // The target position in absolute steps
  int8_t plane_steppers[3];                        // A vector with the steppers of axis_x and axis_y set to 1, the remaining 0
  int incomplete;                                  // True if the arc has not reached its target yet
};

/* The whole state of the motion-control-system in one struct. Makes the code a little bit hard to 
   read, but lets us initialize the state of the system by just clearing a single, contigous block of memory.   
   By overlaying the variables of the different modes in a union we save a few bytes of precious SRAM.
*/
struct MotionControlState {
  int8_t mode;            // The current operation mode
  int32_t position[3];    // The current position of the tool in absolute steps
  int32_t pace;  // Microseconds between each update in the current mode
  uint8_t direction_bits; // The direction bits to be used with any upcoming step-instruction
  union {
    struct LinearMotionParameters linear; // variables used in MC_MODE_LINEAR
    struct ArcMotionParameters arc;       // variables used in MC_MODE_ARC
    uint32_t dwell_milliseconds;          // variable used in MC_MODE_DWELL
  };
};
struct MotionControlState mc;

void set_stepper_directions(int8_t *direction);
inline void step_steppers(uint8_t *enabled);
inline void step_axis(uint8_t axis);
void prepare_linear_motion(uint32_t x, uint32_t y, uint32_t z, float feed_rate, int invert_feed_rate);

void mc_init()
{
	// Initialize state variables
  memset(&mc, 0, sizeof(mc));
}

void mc_dwell(uint32_t milliseconds) 
{
  mc.mode = MC_MODE_DWELL;
  mc.dwell_milliseconds = milliseconds;
}

// Prepare for linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
// unless invert_feed_rate is true. Then the feed_rate states the number of seconds for the whole movement.
void mc_linear_motion(double x, double y, double z, float feed_rate, int invert_feed_rate)
{
  prepare_linear_motion(trunc(x*X_STEPS_PER_MM), trunc(y*Y_STEPS_PER_MM), trunc(z*Z_STEPS_PER_MM), feed_rate, invert_feed_rate);
}

// Same as mc_linear_motion but accepts target in absolute step coordinates
void prepare_linear_motion(uint32_t x, uint32_t y, uint32_t z, float feed_rate, int invert_feed_rate)
{
  memset(&mc.linear, 0, sizeof(mc.arc));
  
  mc.linear.target[X_AXIS] = x;
  mc.linear.target[Y_AXIS] = y;
  mc.linear.target[Z_AXIS] = z;
  
  mc.mode = MC_MODE_LINEAR;
  uint8_t axis; // loop variable
  // Determine direction and travel magnitude for each axis
  for(axis = X_AXIS; axis <= Z_AXIS; axis++) {
  	mc.linear.step_count[axis] = abs(mc.linear.target[axis] - mc.position[axis]);
    mc.linear.direction[axis] = signof(mc.linear.target[axis] - mc.position[axis]);
  }
  // Find the magnitude of the axis with the longest travel
	mc.linear.maximum_steps = max(mc.linear.step_count[Z_AXIS], 
	  max(mc.linear.step_count[X_AXIS], mc.linear.step_count[Y_AXIS]));
	if(mc.linear.maximum_steps == 0) { return; }
  // Nothing to do?
	if ((mc.linear.maximum_steps) == 0) 
	{ 
    mc.mode = MC_MODE_AT_REST;
    return; 
	}
	// Set up a neat counter for each axis
  for(axis = X_AXIS; axis <= Z_AXIS; axis++) {
    mc.linear.counter[axis] = -mc.linear.maximum_steps/2;
  }
	// Set our direction pins 
  set_stepper_directions(mc.linear.direction);
  // Calculate the microseconds we need to wait between each step to achieve the desired feed rate
  if (invert_feed_rate) {
  	mc.pace = 
  	  (feed_rate*1000000)/mc.linear.maximum_steps;	  
  } else {
  	// Ask old Phytagoras to estimate how many mm our next move is going to take us:
  	double millimeters_to_travel = 
      sqrt(pow(X_STEPS_PER_MM*mc.linear.step_count[X_AXIS],2) + 
          pow(Y_STEPS_PER_MM*mc.linear.step_count[Y_AXIS],2) + 
          pow(Z_STEPS_PER_MM*mc.linear.step_count[Z_AXIS],2));
    // Calculate the microseconds between steps that we should wait in order to travel the 
    // designated amount of millimeters in the amount of steps we are going to generate
  	mc.pace = 
      ((millimeters_to_travel * ONE_MINUTE_OF_MICROSECONDS) / feed_rate) / mc.linear.maximum_steps;	
  }
}

void execute_linear_motion()
{
	// Flags to keep track of which axes to step
  uint8_t step[3];
  uint8_t axis; // loop variable
  
  while(mc.mode) {
  	// Trace the line
    clear_vector(step);
    for(axis = X_AXIS; axis <= Z_AXIS; axis++) {
  		if (mc.linear.target[axis] != mc.position[axis])
  		{
  			mc.linear.counter[axis] += mc.linear.step_count[axis];
  			if (mc.linear.counter[axis] > 0)
  			{
          step[axis] = true;
  				mc.linear.counter[axis] -= mc.linear.maximum_steps;
          mc.position[axis] += mc.linear.direction[axis];
  			}
  		}
  	}
  	if (step[X_AXIS] | step[Y_AXIS] | step[Z_AXIS]) {
      step_steppers(step);
  	} else {
      mc.mode = MC_MODE_AT_REST;
  	}
  }
}

// Prepare an arc. theta == start angle, angular_travel == number of radians to go along the arc,
// positive angular_travel means clockwise, negative means counterclockwise. Radius == the radius of the
// circle in millimeters. axis_1 and axis_2 selects the plane in tool space. 
// ISSUE: The arc interpolator assumes all axes have the same steps/mm as the X axis.
void mc_arc(double theta, double angular_travel, double radius, int axis_1, int axis_2, double feed_rate)
{  
  memset(&mc.arc, 0, sizeof(mc.arc));
  uint32_t radius_steps = round(radius*X_STEPS_PER_MM);
	if(radius_steps == 0) { return; }
  mc.mode = MC_MODE_ARC;
  // Determine angular direction (+1 = clockwise, -1 = counterclockwise)
  mc.arc.angular_direction = signof(angular_travel);
  // Calculate the initial position and target position in the local coordinate system of the arc
  mc.arc.x = round(sin(theta)*radius_steps); 
  mc.arc.y = round(cos(theta)*radius_steps);
  mc.arc.target_x = trunc(sin(theta+angular_travel)*radius_steps);
  mc.arc.target_y = trunc(cos(theta+angular_travel)*radius_steps);
  // Precalculate these values to optimize target detection
  mc.arc.target_direction_x = signof(mc.arc.target_x)*mc.arc.angular_direction;
  mc.arc.target_direction_y = signof(mc.arc.target_y)*mc.arc.angular_direction;
  // The "error" factor is kept up to date so that it is always == (x**2+y**2-radius**2). When error 
  // <0 we are inside the arc, when it is >0 we are outside of the arc, and when it is 0 we 
  // are exactly on top of the arc.
  mc.arc.error = mc.arc.x*mc.arc.x + mc.arc.y*mc.arc.y - radius_steps*radius_steps;
  // Because the error-value moves in steps of (+/-)2x+1 and (+/-)2y+1 we save a couple of multiplications
  // by keeping track of the doubles of the arc coordinates at all times.
  mc.arc.x2 = 2*mc.arc.x;
  mc.arc.y2 = 2*mc.arc.y; 
  
  // Set up a vector with the steppers we are going to use tracing the plane of this arc
  mc.arc.plane_steppers[axis_1] = 1;
  mc.arc.plane_steppers[axis_2] = 1;
  // And map the local coordinate system of the arc onto the tool axes of the selected plane
  mc.arc.axis_x = axis_1;
  mc.arc.axis_y = axis_2;
  // The amount of steppings performed while tracing a full circle is equal to the sum of sides in a 
  // square inscribed in the circle. We use this to estimate the amount of steps as if this arc was a full circle:
  uint32_t steps_in_half_circle = round(radius_steps * 4 * (1/sqrt(2)));
  // We then calculate the millimeters of travel along the circumference of that same full circle
  double millimeters_half_circumference = radius*M_PI;
  // Then we calculate the microseconds between each step as if we will trace the full circle.
  // It doesn't matter what fraction of the circle we are actuallyt going to trace. The pace is the same.
	mc.pace = 
    ((millimeters_half_circumference * ONE_MINUTE_OF_MICROSECONDS) / feed_rate) / steps_in_half_circle;	
  mc.arc.incomplete = true;
}

#define check_arc_target \
  if ((mc.arc.x * mc.arc.target_direction_y >= \
          mc.arc.target_x * mc.arc.target_direction_y) && \
         (mc.arc.y * mc.arc.target_direction_x <= \
          mc.arc.target_y * mc.arc.target_direction_x)) \
  { if ((signof(mc.arc.x) == signof(mc.arc.target_x)) && (signof(mc.arc.y) == signof(mc.arc.target_y))) \
    { mc.arc.incomplete = false; } }

// Internal method used by execute_arc to trace horizontally in the general direction provided by dx and dy
void step_arc_along_x(int8_t dx, int8_t dy) 
{
  uint32_t diagonal_error;
  mc.arc.x+=dx;
  mc.arc.error += 1+mc.arc.x2*dx;
  mc.arc.x2 += 2*dx;
  diagonal_error = mc.arc.error + 1 + mc.arc.y2*dy;
  if(abs(mc.arc.error) >= abs(diagonal_error)) {
    mc.arc.y += dy;
    mc.arc.y2 += 2*dy;
    mc.arc.error = diagonal_error;
    step_steppers(mc.arc.plane_steppers); // step diagonal
  } else {
    step_axis(mc.arc.axis_x); // step straight
  }
  check_arc_target;
}

// Internal method used by execute_arc to trace vertically in the general direction provided by dx and dy
void step_arc_along_y(int8_t dx, int8_t dy) 
{  
  uint32_t diagonal_error;
  mc.arc.y+=dy; 
  mc.arc.error += 1+mc.arc.y2*dy; 
  mc.arc.y2 += 2*dy; 
  diagonal_error = mc.arc.error + 1 + mc.arc.x2*dx; 
  if(abs(mc.arc.error) >= abs(diagonal_error)) { 
    mc.arc.x += dx; 
    mc.arc.x2 += 2*dx; 
    mc.arc.error = diagonal_error; 
    step_steppers(mc.arc.plane_steppers); // step diagonal
  } else {
    step_axis(mc.arc.axis_y); // step straight
  }
  check_arc_target;
}

// Will trace the configured arc until the target is reached. 
void execute_arc()
{
  uint32_t start_x = mc.arc.x;
  uint32_t start_y = mc.arc.y;
  int dx, dy; // Trace directions

  // mc.mode is set to 0 (MC_MODE_AT_REST) when target is reached
  while(mc.arc.incomplete)
  {
    dx = (mc.arc.y!=0) ?  signof(mc.arc.y) * mc.arc.angular_direction : -signof(mc.arc.x);
    dy = (mc.arc.x!=0) ? -signof(mc.arc.x) * mc.arc.angular_direction : -signof(mc.arc.y);

    // Take dx and dy which are local to the arc being generated and map them on to the 
    // selected tool-space-axes for the current arc.
    mc.arc.direction[mc.arc.axis_x] = dx;
    mc.arc.direction[mc.arc.axis_y] = dy;
    set_stepper_directions(mc.arc.direction);

    if (abs(mc.arc.x)<abs(mc.arc.y)) {
      step_arc_along_x(dx,dy);    
    } else {      
      step_arc_along_y(dx,dy);    
    }
  }
  
  // Update the tool position to the new actual position
  mc.position[mc.arc.axis_x] += mc.arc.x-start_x;
  mc.position[mc.arc.axis_y] += mc.arc.y-start_y;
  // Todo: Because of rounding errors we might still be off by a step or two. 
  mc.mode = MC_MODE_AT_REST;  
}

void mc_go_home()
{
  mc.mode = MC_MODE_HOME;
}

void execute_go_home() 
{
  st_go_home();
  st_synchronize();
  clear_vector(mc.position); // By definition this is location [0, 0, 0]
  mc.mode = MC_MODE_AT_REST;
}

void mc_execute() {
  st_set_pace(mc.pace);
  sp_send_execution_marker();
  while(mc.mode) { // Loop because one task might start another task
    switch(mc.mode) {
      case MC_MODE_AT_REST: break;
      case MC_MODE_DWELL: st_synchronize(); _delay_ms(mc.dwell_milliseconds); mc.mode = MC_MODE_AT_REST; break;
      case MC_MODE_LINEAR: execute_linear_motion(); break;
      case MC_MODE_ARC: execute_arc(); break;
      case MC_MODE_HOME: execute_go_home(); break;
    }
  }
}

int mc_status() 
{
  return(mc.mode);
}

// Set the direction pins for the stepper motors according to the provided vector. 
// direction is an array of three 8 bit integers representing the direction of 
// each motor. The values should be -1 (reverse), 0 or 1 (forward).
void set_stepper_directions(int8_t *direction) 
{
  /* Sorry about this convoluted code! It uses the fact that bit 7 of each direction
     int is set when the direction == -1, but is 0 when direction is forward. This
     way we can generate the whole direction bit-mask without doing any comparisions
     or branching. Fast and compact, yet practically unreadable. Sorry sorry sorry.
  */
  mc.direction_bits = (
    ((direction[X_AXIS]&0x80)>>(7-X_DIRECTION_BIT)) |
    ((direction[Y_AXIS]&0x80)>>(7-Y_DIRECTION_BIT)) |
    ((direction[Z_AXIS]&0x80)>>(7-Z_DIRECTION_BIT)));
}

// Step enabled steppers. Enabled should be an array of three bytes. Each byte represent one
// stepper motor in the order X, Y, Z. Set the bytes of the steppers you want to step to
// 1, and the rest to 0. 
inline void step_steppers(uint8_t *enabled) 
{
  st_buffer_step(mc.direction_bits | (enabled[X_AXIS]<<X_STEP_BIT) | 
    (enabled[Y_AXIS]<<Y_STEP_BIT) | (enabled[Z_AXIS]<<Z_STEP_BIT));
}

// Step only one motor
inline void step_axis(uint8_t axis) 
{
  switch (axis) {
    case X_AXIS: st_buffer_step(mc.direction_bits | (1<<X_STEP_BIT)); break;
    case Y_AXIS: st_buffer_step(mc.direction_bits | (1<<Y_STEP_BIT)); break;
    case Z_AXIS: st_buffer_step(mc.direction_bits | (1<<Z_STEP_BIT)); break;
  }
}

// Wait until all operations are completed
void mc_wait()
{
  st_synchronize();
}

