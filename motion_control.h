/*
  motion_control.h - high level interface for issuing motion commands
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

#ifndef motion_control_h
#define motion_control_h

#include <avr/io.h>
#include "planner.h"

// NOTE: Although the following functions structurally belongs in this module, there is nothing to do but
// to forward the request to the planner.

// Execute linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
// unless invert_feed_rate is true. Then the feed_rate means that the motion should be completed in
// (1 minute)/feed_rate time.
#define mc_line(x, y, z, feed_rate, invert_feed_rate) plan_buffer_line(x, y, z, feed_rate, invert_feed_rate) 

#define mc_set_current_position(x, y, z) plan_set_current_position(x, y, z) 

#ifdef __AVR_ATmega328P__
// Execute an arc. theta == start angle, angular_travel == number of radians to go along the arc,
// positive angular_travel means clockwise, negative means counterclockwise. Radius == the radius of the
// circle in millimeters. axis_1 and axis_2 selects the circle plane in tool space. Stick the remaining
// axis in axis_l which will be the axis for linear travel if you are tracing a helical motion.

void mc_arc(double theta, double angular_travel, double radius, double linear_travel, int axis_1, int axis_2, 
  int axis_linear, double feed_rate, int invert_feed_rate, double *position);
#endif
  
// Dwell for a couple of time units
void mc_dwell(uint32_t milliseconds);

// Send the tool home (not implemented)
void mc_go_home();

#endif
