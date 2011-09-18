/*
  motion_control.h - high level interface for issuing motion commands
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Sungeun K. Jeon
  
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
// Execute an arc in offset mode format. position == current xyz, target == target xyz, 
// offset == offset from current xyz, axis_XXX defines circle plane in tool space, axis_linear is
// the direction of helical travel, radius == circle radius, isclockwise boolean. Used
// for vector transformation direction.
void mc_arc(double *position, double *target, double *offset, uint8_t axis_0, uint8_t axis_1,
  uint8_t axis_linear, double feed_rate, uint8_t invert_feed_rate, double radius, uint8_t isclockwise);
#endif
  
// Dwell for a specific number of seconds
void mc_dwell(double seconds);

// Send the tool home (not implemented)
void mc_go_home();

#endif
