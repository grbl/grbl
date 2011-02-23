/*
  motion_control.c - high level interface for issuing motion commands
  Part of Grbl

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

#include <avr/io.h>
#include "settings.h"
#include "motion_control.h"
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#include "nuts_bolts.h"
#include "stepper.h"
#include "planner.h"
#include "wiring_serial.h"


void mc_dwell(uint32_t milliseconds) 
{
  st_synchronize();
  _delay_ms(milliseconds);
}

// Execute an arc. theta == start angle, angular_travel == number of radians to go along the arc,
// positive angular_travel means clockwise, negative means counterclockwise. Radius == the radius of the
// circle in millimeters. axis_1 and axis_2 selects the circle plane in tool space. Stick the remaining
// axis in axis_l which will be the axis for linear travel if you are tracing a helical motion.
// position is a pointer to a vector representing the current position in millimeters.

#ifdef __AVR_ATmega328P__
// The arc is approximated by generating a huge number of tiny, linear segments. The length of each 
// segment is configured in settings.mm_per_arc_segment.  
void mc_arc(double theta, double angular_travel, double radius, double linear_travel, int axis_1, int axis_2, 
  int axis_linear, double feed_rate, int invert_feed_rate, double *position)
{      
  int acceleration_manager_was_enabled = plan_is_acceleration_manager_enabled();
  plan_set_acceleration_manager_enabled(false); // disable acceleration management for the duration of the arc
  double millimeters_of_travel = hypot(angular_travel*radius, labs(linear_travel));
  if (millimeters_of_travel == 0.0) { return; }
  uint16_t segments = round(millimeters_of_travel/settings.mm_per_arc_segment);
  // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
  // by a number of discrete segments. The inverse feed_rate should be correct for the sum of 
  // all segments.
  if (invert_feed_rate) { feed_rate *= segments; }
  // The angular motion for each segment
  double theta_per_segment = angular_travel/segments;
  // The linear motion for each segment
  double linear_per_segment = linear_travel/segments;
  // Compute the center of this circle
  double center_x = position[axis_1]-sin(theta)*radius;
  double center_y = position[axis_2]-cos(theta)*radius;
  // a vector to track the end point of each segment
  double target[3];
  int i;
  // Initialize the linear axis
  target[axis_linear] = position[axis_linear];
  for (i=0; i<segments; i++) {
    target[axis_linear] += linear_per_segment;
    theta += theta_per_segment;
    target[axis_1] = center_x+sin(theta)*radius;
    target[axis_2] = center_y+cos(theta)*radius;
    plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], feed_rate, invert_feed_rate);
  }
  plan_set_acceleration_manager_enabled(acceleration_manager_was_enabled);
}
#endif

void mc_go_home()
{
  st_go_home();
}
