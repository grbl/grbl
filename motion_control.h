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

#define MC_MODE_AT_REST 0
#define MC_MODE_LINEAR 1
#define MC_MODE_ARC 2
#define MC_MODE_DWELL 3
#define MC_MODE_HOME 4

// Initializes the motion_control subsystem resources
void mc_init();

// Prepare for linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
// unless invert_feed_rate is true. Then the feed_rate states the number of seconds for the whole movement.
void mc_line(double x, double y, double z, float feed_rate, int invert_feed_rate);

// Prepare an arc. theta == start angle, angular_travel == number of radians to go along the arc,
// positive angular_travel means clockwise, negative means counterclockwise. Radius == the radius of the
// circle in millimeters. axis_1 and axis_2 selects the plane in tool space. 
// Known issue: This method pretends that all axes uses the same steps/mm as the X axis. Which might
// not be the case ... (To be continued)
void mc_arc(double theta, double angular_travel, double radius, int axis_1, int axis_2, double feed_rate);

// Prepare linear motion relative to the current position.
void mc_dwell(uint32_t milliseconds);

// Prepare to send the tool position home
void mc_go_home();

// Start the prepared operation. In the current implementation this will block for most of the task at hand.
// In future implementations it might not block at all. If you want to make sure the system has reached 
// quiescence call mc_wait()
void mc_execute();

// Wait until all operations complete
void mc_wait();

// Check motion control status. result == 0: the system is idle. result > 0: the system is busy,
// result < 0: the system is in an error state.
int mc_status();

void printCurrentPosition();

#endif
