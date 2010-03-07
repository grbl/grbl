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

#include <avr/io.h>
#include "config.h"
#include "motion_control.h"
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#include "nuts_bolts.h"
#include "stepper.h"

#include "wiring_serial.h"

int32_t position[3];    // The current position of the tool in absolute steps

void mc_init()
{
  clear_vector(position);
}

void mc_dwell(uint32_t milliseconds) 
{
  st_synchronize();
  _delay_ms(milliseconds);
}

// Execute linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
// unless invert_feed_rate is true. Then the feed_rate means that the motion should be completed in
// 1/feed_rate minutes.
void mc_line(double x, double y, double z, float feed_rate, int invert_feed_rate)
{
  uint8_t axis; // loop variable
  int32_t target[3]; // The target position in absolute steps
  int32_t steps[3]; // The target line in relative steps
  
  target[X_AXIS] = lround(x*settings.steps_per_mm[0]);
  target[Y_AXIS] = lround(y*settings.steps_per_mm[1]);
  target[Z_AXIS] = lround(z*settings.steps_per_mm[2]); 

  for(axis = X_AXIS; axis <= Z_AXIS; axis++) {
    steps[axis] = target[axis]-position[axis];
  }
  
	if (invert_feed_rate) {
    st_buffer_line(steps[X_AXIS], steps[Y_AXIS], steps[Z_AXIS], lround(ONE_MINUTE_OF_MICROSECONDS/feed_rate));
	} else {
  	// Ask old Phytagoras to estimate how many mm our next move is going to take us
  	double millimeters_of_travel = sqrt(
  	  square(steps[X_AXIS]/settings.steps_per_mm[0]) + 
  	  square(steps[Y_AXIS]/settings.steps_per_mm[1]) + 
  	  square(steps[Z_AXIS]/settings.steps_per_mm[2]));
    st_buffer_line(steps[X_AXIS], steps[Y_AXIS], steps[Z_AXIS],
      lround((millimeters_of_travel/feed_rate)*1000000));
	}
	memcpy(position, target, sizeof(target)); // position[] = target[] 
}

// Execute an arc. theta == start angle, angular_travel == number of radians to go along the arc,
// positive angular_travel means clockwise, negative means counterclockwise. Radius == the radius of the
// circle in millimeters. axis_1 and axis_2 selects the circle plane in tool space. Stick the remaining
// axis in axis_l which will be the axis for linear travel if you are tracing a helical motion.

// The arc is approximated by generating a huge number of tiny, linear segments. The length of each 
// segment is configured in config.h by setting MM_PER_ARC_SEGMENT.  
void mc_arc(double theta, double angular_travel, double radius, double linear_travel, int axis_1, int axis_2, 
  int axis_linear, double feed_rate, int invert_feed_rate)
{
  double millimeters_of_travel = hypot(angular_travel*radius, labs(linear_travel));
  if (millimeters_of_travel == 0.0) { return; }
  uint16_t segments = ceil(millimeters_of_travel/settings.mm_per_arc_segment);
  // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
  // by a number of discrete segments. The inverse feed_rate should be correct for the sum of 
  // all segments.
  if (invert_feed_rate) { feed_rate *= segments; }
  // The angular motion for each segment
  double theta_per_segment = angular_travel/segments;
  // The linear motion for each segment
  double linear_per_segment = linear_travel/segments;
  // Compute the center of this circle
  double center_x = (position[axis_1]/settings.steps_per_mm[axis_1])-sin(theta)*radius;
  double center_y = (position[axis_2]/settings.steps_per_mm[axis_2])-cos(theta)*radius;
  // a vector to track the end point of each segment
  double target[3];
  int i;
  // Initialize the linear axis
  target[axis_linear] = position[axis_linear]/Z_STEPS_PER_MM;
  for (i=0; i<=segments; i++) {
    target[axis_linear] += linear_per_segment;
    theta += theta_per_segment;
    target[axis_1] = center_x+sin(theta)*radius;
    target[axis_2] = center_y+cos(theta)*radius;
    mc_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], feed_rate, invert_feed_rate);
  }
}

void mc_go_home()
{
  st_go_home();
  clear_vector(position); // By definition this is location [0, 0, 0]
}
