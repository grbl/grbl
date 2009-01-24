/*
  motion_control.h - cartesian robot controller.
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

#ifndef motion_control_h
#define motion_control_h 

#include <avr/io.h>

/* All coordinates are in step-counts. */

// Initializes the motion_control subsystem resources
void mc_init();

// Prepare for linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
// unless invert_feed_rate is true. Then the feed_rate states the number of seconds for the whole movement.
void mc_linear_motion(double x, double y, double z, float feed_rate, int invert_feed_rate);
// Prepare linear motion relative to the current position.
void mc_dwell(uint32_t milliseconds);
// Prepare to send the tool position home
void mc_go_home();
// Start the prepared operation.
void mc_execute();

// Block until the motion control system is idle
void mc_wait();


// Check motion control status. result == 0: the system is idle. result > 0: the system is busy,
// result < 0: the system is in an error state.
int mc_status();

#endif
