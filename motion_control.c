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

// Execute linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
// unless invert_feed_rate is true. Then the feed_rate states the number of seconds for the whole movement.
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
  
  
  // Setup

  target[X_AXIS] = x*X_STEPS_PER_MM;
  target[Y_AXIS] = y*Y_STEPS_PER_MM;
  target[Z_AXIS] = z*Z_STEPS_PER_MM;
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
  // Calculate the microseconds we need to wait between each step to achieve the desired feed rate
  if (invert_feed_rate) {
  	st_buffer_pace((feed_rate*1000000)/maximum_steps);	  
  } else {
  	// Ask old Phytagoras to estimate how many mm our next move is going to take us:
  	double millimeters_to_travel = 
      sqrt(pow(X_STEPS_PER_MM*step_count[X_AXIS],2) + 
          pow(Y_STEPS_PER_MM*step_count[Y_AXIS],2) + 
          pow(Z_STEPS_PER_MM*step_count[Z_AXIS],2));
    // Calculate the microseconds between steps that we should wait in order to travel the 
    // designated amount of millimeters in the amount of steps we are going to generate
  	st_buffer_pace(((millimeters_to_travel * ONE_MINUTE_OF_MICROSECONDS) / feed_rate) / maximum_steps);	
  }
  
  // Execution

  mode = MC_MODE_LINEAR;
  
  while(mode) {
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
  	if (step_bits) {
      step_steppers(step_bits);
  	} else {
      mode = MC_MODE_AT_REST;
  	}
  }
}


// Execute an arc. theta == start angle, angular_travel == number of radians to go along the arc,
// positive angular_travel means clockwise, negative means counterclockwise. Radius == the radius of the
// circle in millimeters. axis_1 and axis_2 selects the plane in tool space. 
// ISSUE: The arc interpolator assumes all axes have the same steps/mm as the X axis.
void mc_arc(double theta, double angular_travel, double radius, int axis_1, int axis_2, double feed_rate)
{  
  uint32_t start_x, start_y;
  uint32_t diagonal_error;

  int8_t direction[3];                             // The direction of travel along each axis (-1, 0 or 1)
  int8_t angular_direction;                        // 1 = clockwise, -1 = anticlockwise
  int32_t x, y, target_x, target_y;                // current position and target position in the 
                                                   // local coordinate system of the arc-generator where [0,0] is the 
                                                   // center of the arc.
  int target_direction_x, target_direction_y;      // signof(target_x)*angular_direction precalculated for speed                                                 
  int32_t error, x2, y2;                           // error is always == (x**2 + y**2 - radius**2), 
                                                   // x2 is always 2*x, y2 is always 2*y
  uint8_t axis_x, axis_y;                          // maps the arc axes to stepper axes
  int8_t diagonal_bits;                            // A bitmask with the stepper bits for both selected axes set
  int incomplete;                                  // True if the arc has not reached its target yet
  
  int dx, dy; // Trace directions

  // Setup
  
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
  // Because the error-value moves in steps of (+/-)2x+1 and (+/-)2y+1 we save a couple of multiplications
  // by keeping track of the doubles of the arc coordinates at all times.
  x2 = 2*x;
  y2 = 2*y;   
  // Set up a vector with the steppers we are going to use tracing the plane of this arc
  diagonal_bits = st_bit_for_stepper(axis_1);
  diagonal_bits |= st_bit_for_stepper(axis_2);
  // And map the local coordinate system of the arc onto the tool axes of the selected plane
  axis_x = axis_1;
  axis_y = axis_2;
  // The amount of steppings performed while tracing a half circle is equal to the sum of sides in a 
  // square inscribed in the circle. We use this to estimate the amount of steps as if this arc was a half circle:
  uint32_t steps_in_half_circle = round(radius_steps * 4 * (1/sqrt(2)));
  // We then calculate the millimeters of travel along the circumference of that same half circle
  double millimeters_half_circumference = radius*M_PI;
  // Then we calculate the microseconds between each step as if we will trace the full circle.
  // It doesn't matter what fraction of the circle we are actually going to trace. The pace is the same.
  st_buffer_pace(((millimeters_half_circumference * ONE_MINUTE_OF_MICROSECONDS) / feed_rate) / steps_in_half_circle);    

  // Execution

  mode = MC_MODE_ARC;
  
  incomplete = true;
  while(incomplete)
  {
    dx = (y!=0) ?  signof(y) * angular_direction : -signof(x);
    dy = (x!=0) ? -signof(x) * angular_direction : -signof(y);
    // Take dx and dy which are local to the arc being generated and map them on to the 
    // selected tool-space-axes for the current arc.
    direction[axis_x] = dx;
    direction[axis_y] = dy;
    set_stepper_directions(direction);
    // Check which axis will be "major" for this stepping
    if (abs(x)<abs(y)) {
      // Step arc horizontally      
      error += 1+x2*dx;
      x+=dx; x2 += 2*dx;
      diagonal_error = error + 1 + y2*dy;
      if(abs(error) >= abs(diagonal_error)) {
        y += dy; y2 += 2*dy;
        error = diagonal_error;
        step_steppers(diagonal_bits); // step diagonal
      } else {
        step_axis(axis_x); // step straight
      }
    } else {      
      // Step arc vertically      
      error += 1+y2*dy; 
      y+=dy; y2 += 2*dy; 
      diagonal_error = error + 1 + x2*dx; 
      if(abs(error) >= abs(diagonal_error)) { 
        x += dx; x2 += 2*dx; 
        error = diagonal_error; 
        step_steppers(diagonal_bits); // step diagonal
      } else {
        step_axis(axis_y); // step straight
      }
    }    
    // Check if target has been reached. Todo: Simplify/optimize/clarify
    if ((x * target_direction_y >= 
            target_x * target_direction_y) && 
           (y * target_direction_x <= 
            target_y * target_direction_x)) 
    { if ((signof(x) == signof(target_x)) && (signof(y) == signof(target_y))) 
      { incomplete = false; } }    
  }  
  // Update the tool position to the new actual position
  position[axis_x] += x-start_x;
  position[axis_y] += y-start_y;
  mode = MC_MODE_AT_REST;
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
// each motor. The values should be -1 (reverse), 0 or 1 (forward).
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
