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
  int32_t target[3]; // The target position in absolute steps
  
  // Setup ---------------------------------------------------------------------------------------------------
  PORTD |= (1<<4);
  PORTD |= (1<<5);
  target[X_AXIS] = round(x*X_STEPS_PER_MM);
  target[Y_AXIS] = round(y*Y_STEPS_PER_MM);
  target[Z_AXIS] = round(z*Z_STEPS_PER_MM); 
  PORTD ^= (1<<5);
  // Determine direction and travel magnitude for each axis
  for(axis = X_AXIS; axis <= Z_AXIS; axis++) {
  	step_count[axis] = labs(target[axis] - position[axis]);
    direction[axis] = signof(target[axis] - position[axis]);
  }
  PORTD ^= (1<<5);
  // Find the magnitude of the axis with the longest travel
	maximum_steps = max(step_count[Z_AXIS], 
	  max(step_count[X_AXIS], step_count[Y_AXIS]));
  PORTD ^= (1<<5);
  // Nothing to do?
  if (maximum_steps == 0) { PORTD &= ~(1<<4); PORTD |= (1<<5); return; }
  PORTD ^= (1<<5);
	// Set up a neat counter for each axis
  for(axis = X_AXIS; axis <= Z_AXIS; axis++) {
    counter[axis] = -maximum_steps/2;
  }
  PORTD ^= (1<<5);
	// Set our direction pins 
  set_stepper_directions(direction);
  PORTD ^= (1<<5);

	// Ask old Phytagoras to estimate how many mm our next move is going to take us
	double millimeters_of_travel = 
    sqrt(square(X_STEPS_PER_MM*step_count[X_AXIS]) + 
        square(Y_STEPS_PER_MM*step_count[Y_AXIS]) + 
        square(Z_STEPS_PER_MM*step_count[Z_AXIS]));
      PORTD ^= (1<<5);
  // And set the step pace
  compute_and_set_step_pace(feed_rate, millimeters_of_travel, maximum_steps, invert_feed_rate);
  PORTD &= ~(1<<5);
  PORTD &= ~(1<<4);
  
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
    if(step_bits) { 
      step_steppers(step_bits);
    }
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

inline void step_steppers(uint8_t bits) 
{
  st_buffer_step(direction_bits | bits);
}
