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
#include "geometry.h"

#include "wiring_serial.h"

#define ONE_MINUTE_OF_MICROSECONDS 60000000.0

volatile int8_t mode;   // The current operation mode
int32_t position[3];    // The current position of the tool in absolute steps
uint8_t direction_bits; // The direction bits to be used with any upcoming step-instruction

void set_stepper_directions(int8_t *direction);
inline void step_steppers(uint8_t bits);
inline void step_axis(uint8_t axis);
void prepare_linear_motion(uint32_t x, uint32_t y, uint32_t z, float feed_rate, int invert_feed_rate);

void mc_init()
{
  mode = MC_MODE_AT_REST;
  clear_vector(position);
}

void mc_dwell(uint32_t milliseconds) 
{
  mode = MC_MODE_DWELL;
  st_synchronize(); 
  _delay_ms(milliseconds); 
  mode = MC_MODE_AT_REST; 
}

// Calculate the microseconds between steps that we should wait in order to travel the 
// designated amount of millimeters in the amount of steps we are going to generate
void compute_and_set_step_pace(double feed_rate, double millimeters_of_travel, uint32_t steps, int invert) {
  int32_t pace;
  if (invert) {
    pace = round(ONE_MINUTE_OF_MICROSECONDS/feed_rate/steps);
  } else {
    pace = round((ONE_MINUTE_OF_MICROSECONDS/X_STEPS_PER_MM)/feed_rate);
  }
  st_buffer_pace(pace);
}

// Execute linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
// unless invert_feed_rate is true. Then the feed_rate means that the motion should be completed in
// 1/feed_rate minutes.
void mc_line(double x, double y, double z, float feed_rate, int invert_feed_rate)
{
	// Flags to keep track of which axes to step
  uint8_t step_bits;
  uint8_t axis; // loop variable
  int8_t direction[3];  // The direction of travel along each axis (-1, 0 or 1)
  int32_t target[3], // The target position in absolute steps
    step_count[3],   // Absolute steps of travel along each axis
    counter[3],      // A counter used in the bresenham algorithm for line plotting
    maximum_steps;   // The larges absolute step-count of any axis  
  
  // Setup ---------------------------------------------------------------------------------------------------

  target[X_AXIS] = round(x*X_STEPS_PER_MM);
  target[Y_AXIS] = round(y*Y_STEPS_PER_MM);
  target[Z_AXIS] = round(z*Z_STEPS_PER_MM);  
  // Determine direction and travel magnitude for each axis
  for(axis = X_AXIS; axis <= Z_AXIS; axis++) {
  	step_count[axis] = abs(target[axis] - position[axis]);
    direction[axis] = signof(target[axis] - position[axis]);
  }
  // Find the magnitude of the axis with the longest travel
	maximum_steps = max(step_count[Z_AXIS], 
	  max(step_count[X_AXIS], step_count[Y_AXIS]));
  // Nothing to do?
	if (maximum_steps == 0) { return; }
	// Set up a neat counter for each axis
  for(axis = X_AXIS; axis <= Z_AXIS; axis++) {
    counter[axis] = -maximum_steps/2;
  }
	// Set our direction pins 
  set_stepper_directions(direction);

	// Ask old Phytagoras to estimate how many mm our next move is going to take us
	double millimeters_of_travel = 
    sqrt(square(X_STEPS_PER_MM*step_count[X_AXIS]) + 
        square(Y_STEPS_PER_MM*step_count[Y_AXIS]) + 
        square(Z_STEPS_PER_MM*step_count[Z_AXIS]));
  // And set the step pace
  compute_and_set_step_pace(feed_rate, millimeters_of_travel, maximum_steps, invert_feed_rate);
  
  // Execution -----------------------------------------------------------------------------------------------

  mode = MC_MODE_LINEAR;
  
  do {
  	// Trace the line
    step_bits = 0;
    for(axis = X_AXIS; axis <= Z_AXIS; axis++) {
  		if (target[axis] != position[axis])
  		{
  			counter[axis] += step_count[axis];
  			if (counter[axis] > 0)
  			{
          step_bits |= st_bit_for_stepper(axis);
  				counter[axis] -= maximum_steps;
          position[axis] += direction[axis];
  			}
  		}
  	}
    if(step_bits) { step_steppers(step_bits); }
  } while (step_bits);
  mode = MC_MODE_AT_REST;
}


// Execute an arc. theta == start angle, angular_travel == number of radians to go along the arc,
// positive angular_travel means clockwise, negative means counterclockwise. Radius == the radius of the
// circle in millimeters. axis_1 and axis_2 selects the circle plane in tool space. Stick the remaining
// axis in axis_l which will be the axis for linear travel if you are tracing a helical motion.
// ISSUE: The arc interpolator assumes all axes have the same steps/mm as the X axis.
void mc_arc(double theta, double angular_travel, double radius, double linear_travel, int axis_1, int axis_2, 
  int axis_linear, double feed_rate, int invert_feed_rate)
{  
  uint32_t start_x, start_y;                       // The start position in the coordinate system local to the circle
  uint32_t diagonal_error;                         // A variable to keep track of varations in the error-value during
                                                   // the tracing of the arc

  int8_t direction[3];                             // The direction of travel along each axis (-1, 0 or 1)
  int8_t angular_direction;                        // 1 = clockwise, -1 = anticlockwise
  int32_t x, y, target_x, target_y;                // current position and target position in the 
                                                   // local coordinate system of the arc-generator where [0,0] is the 
                                                   // center of the arc.
  int target_direction_x, target_direction_y;      // signof(target_x)*angular_direction precalculated for speed                                                 
  int32_t error;                                   // error is always == (x**2 + y**2 - radius**2),                                                    

  int dx, dy; // Trace directions

  // Setup arc interpolation --------------------------------------------------------------------------------

  uint32_t radius_steps = round(radius*X_STEPS_PER_MM);
	if(radius_steps == 0) { return; }
  // Determine angular direction (+1 = clockwise, -1 = counterclockwise)
  angular_direction = signof(angular_travel);
  // Calculate the initial position and target position in the local coordinate system of the arc
  start_x = x = round(sin(theta)*radius_steps); 
  start_y = y = round(cos(theta)*radius_steps);
  target_x = trunc(sin(theta+angular_travel)*radius_steps);
  target_y = trunc(cos(theta+angular_travel)*radius_steps);
  // Precalculate these values to optimize target detection
  target_direction_x = signof(target_x)*angular_direction;
  target_direction_y = signof(target_y)*angular_direction;
  // The "error" factor is kept up to date so that it is always == (x**2+y**2-radius**2). When error 
  // <0 we are inside the arc, when it is >0 we are outside of the arc, and when it is 0 we 
  // are exactly on top of the arc.
  error = x*x + y*y - radius_steps*radius_steps;
  
  // Estimate length of arc in steps -------------------------------------------------------------------------
  
  /*  
    To support helical motion we need to know in advance how many steppings the arc will need. 
    The calculations are based on the fact that we trace the circle by offsetting a square. The circle has
    four "sides" or quadrants. For each quadrant we step mainly in one axis. The amount steps for one quarter of the 
    circle (e.g. along the x axis with positive y) is equal to one side of a square inscribed in the circle we
    are tracing.

    Quadrants of the circle
    
    +---- 0 ----+    0 - y is always positive and |x| < |y|
    |           |    1 - x is always positive and |x| > |y|
    |           |    2 - y is always negative and |x| < |y|
    3     +     1    3 - x is always negative and |x| > |y|
    |           |
    |           |    length of one side: 2*radius/sqrt(2)
    +---- 2 ----+                    
  */

  // Find the quadrants of the starting point and the target
  int start_quadrant = quadrant_of_the_circle(start_x, start_y);
  int target_quadrant = quadrant_of_the_circle(target_x, target_y);
  uint32_t arc_steps=0;
  // Will this whole arc take place within the same quadrant?
  if (start_quadrant == target_quadrant && (abs(angular_travel) <= (M_PI/2))) { 
    if(quadrant_horizontal(start_quadrant)) { // a horizontal quadrant where x will be the primary direction
      arc_steps = abs(target_x-start_x);
    } else { // a vertical quadrant where y will be the primary direction
      arc_steps = abs(target_y-start_y);
    }
  } else { // the start and target points are in different quadrants
    // Lets estimate the amount of steps along half a quadrant
    uint32_t steps_in_half_quadrant = ceil(radius_steps/sqrt(2));
    // Add the steps in the first partial quadrant
    arc_steps += steps_in_partial_quadrant(start_x, start_y, 
      start_quadrant, angular_direction, steps_in_half_quadrant);
    // Count the number of full quadrants between the start and end quadrants
    uint8_t full_quadrants_traveled = full_quadrants_between(start_quadrant, target_quadrant, angular_direction);
    // Add steps for the full quadrants plus some stray steps for "corners"
    arc_steps += full_quadrants_traveled*(steps_in_half_quadrant*2+1);
    // Add the steps in the final partial quadrant. By inverting the angular direction we get the correct number for
    // the target quadrant which steps through the opposite part of the quadrant with respect to the start quadrant.
    arc_steps += steps_in_partial_quadrant(target_x, target_y, 
      target_quadrant, -angular_direction, steps_in_half_quadrant);
  }
  
  // Set up the linear interpolation of the "depth" axis -----------------------------------------------------
  
  int32_t linear_steps = abs(st_millimeters_to_steps(linear_travel, axis_linear));
  int linear_direction = signof(linear_travel);
  // The number of steppings needed to trace this motion is equal to the motion that require the maximum 
  // amount of steps: the arc or the line:
  int32_t maximum_steps = max(linear_steps, arc_steps);
  // Initialize the counters to do 2D linear bresenham as if the motion along the arc itself was a single axis 
  // of the line, while the linear "depth" axis was the other.
  int32_t linear_counter = -maximum_steps/2;
  int32_t arc_counter = -maximum_steps/2;

  // Calculate feed rate -------------------------------------------------------------------------------------
  
  // We then calculate the millimeters of helical travel
  double millimeters_of_travel = hypot(angular_travel*radius, abs(linear_travel));
  // Then we calculate the microseconds between each step as if we will trace the full circle.
  // It doesn't matter what fraction of the circle we are actually going to trace. The pace is the same.
  compute_and_set_step_pace(feed_rate, millimeters_of_travel, maximum_steps, invert_feed_rate);

  // Execution -----------------------------------------------------------------------------------------------

  mode = MC_MODE_ARC;
  // Set the direction of the linear or "depth" axis, cause it will never change
  direction[axis_linear] = linear_direction;
  // Cache some stepper bit-masks to speed up the interpolation code
  uint8_t axis_1_bit = st_bit_for_stepper(axis_1);
  uint8_t axis_2_bit = st_bit_for_stepper(axis_2);
  uint8_t axis_linear_bit = st_bit_for_stepper(axis_linear);
  uint8_t diagonal_bits = (axis_1_bit | axis_2_bit);
  
  uint8_t step_bits;
  
  while(mode)
  {
    // This loop sets the bits in the step_bits variable for each stepper it wants to step in this cycle.
    step_bits = 0;    
    // The bresenham algorithm chooses when to travel in the depth axis and when to travel along the arc
    linear_counter += linear_steps;
    if (linear_counter > 0) {
      linear_counter -= maximum_steps;
      // Move one step in the depth direction:
      step_bits |= axis_linear_bit;
    }    
    arc_counter += arc_steps;
    if (arc_counter > 0) {
      arc_counter -= maximum_steps;
      // Do one step of the arc:
      // Determine directions for each axis at this point in the arc
      dx = (y!=0) ?  signof(y) * angular_direction : -signof(x);
      dy = (x!=0) ? -signof(x) * angular_direction : -signof(y);
      // Take dx and dy which are local to the arc being generated and map them on to the 
      // selected tool-space-axes for the current arc.
      direction[axis_1] = dx;
      direction[axis_2] = dy;    
      // Check which axis will be "major" for this stepping
      if (abs(x)<abs(y)) {
        // X is major: Step arc horizontally      
        error += 1 + 2*x * dx;
        x+=dx;
        diagonal_error = error + 1 + 2*y*dy;
        if(abs(error) >= abs(diagonal_error)) {
          y += dy;
          error = diagonal_error;
          step_bits |= diagonal_bits; // step diagonal
        } else {
          step_bits |= axis_1_bit; // step straight
        }
      } else {      
        // Y is major: Step arc vertically      
        error += 1 + 2*y * dy; 
        y+=dy;
        diagonal_error = error + 1 + 2*x * dx; 
        if(abs(error) >= abs(diagonal_error)) { 
          x += dx; 
          error = diagonal_error; 
          step_bits |= diagonal_bits; // step diagonal
        } else {
          step_bits |= axis_2_bit; // step straight
        }
      }
    }    
    // Tell the steppers to do the stepping 
    set_stepper_directions(direction);
    step_steppers(step_bits);
    
    // Check if target has been reached. Todo: Simplify/optimize/clarify
    if ((x * target_direction_y >= 
            target_x * target_direction_y) && 
           (y * target_direction_x <= 
            target_y * target_direction_x)) 
    { if ((signof(x) == signof(target_x)) && (signof(y) == signof(target_y))) 
      { mode = MC_MODE_AT_REST; } }    
  }  
  // Update the tool position to the new actual position
  position[axis_1] += x-start_x;
  position[axis_2] += y-start_y;
  position[axis_2] += linear_steps*linear_direction;  
}

void mc_go_home()
{
  mode = MC_MODE_HOME;
  st_go_home();
  st_synchronize();
  clear_vector(position); // By definition this is location [0, 0, 0]
  mode = MC_MODE_AT_REST;
}

int mc_status() 
{
  return(mode);
}

// Set the direction bits for the stepper motors according to the provided vector. 
// direction is an array of three 8 bit integers representing the direction of 
// each motor. The values should be negative (reverse), 0 or positive (forward).
void set_stepper_directions(int8_t *direction) 
{
  /* Sorry about this convoluted code! It uses the fact that bit 7 of each direction
     int is set when the direction == -1, but is 0 when direction is forward. This
     way we can generate the whole direction bit-mask without doing any comparisions
     or branching. Fast and compact, yet practically unreadable. Sorry sorry sorry.
  */
  direction_bits = (
    ((direction[X_AXIS]&0x80)>>(7-X_DIRECTION_BIT)) |
    ((direction[Y_AXIS]&0x80)>>(7-Y_DIRECTION_BIT)) |
    ((direction[Z_AXIS]&0x80)>>(7-Z_DIRECTION_BIT)));
}

// Step enabled steppers. Enabled should be an array of three bytes. Each byte represent one
// stepper motor in the order X, Y, Z. Set the bytes of the steppers you want to step to
// 1, and the rest to 0. 
inline void step_steppers(uint8_t bits) 
{
  st_buffer_step(direction_bits | bits);
}

// Step only one motor
inline void step_axis(uint8_t axis) 
{
  st_buffer_step(direction_bits | st_bit_for_stepper(axis));
}
