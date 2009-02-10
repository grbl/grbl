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

// Execute linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
// unless invert_feed_rate is true. Then the feed_rate means that the motion should be completed in
// 1/feed_rate minutes.
void mc_line(double x, double y, double z, float feed_rate, int invert_feed_rate);

// Prepare an arc. theta == start angle, angular_travel == number of radians to go along the arc,
// positive angular_travel means clockwise, negative means counterclockwise. Radius == the radius of the
// circle in millimeters. axis_1 and axis_2 selects the plane in tool space. 
// Known issue: This method pretends that all axes uses the same steps/mm as the X axis. Which might
// not be the case ... (To be continued) 
// Regarding feed rate see note on mc_line.
void mc_arc(double theta, double angular_travel, double radius, double linear_travel, int axis_1, int axis_2, 
  int axis_linear, double feed_rate, int invert_feed_rate);

// Dwell for a couple of time units
void mc_dwell(uint32_t milliseconds);

// Send the tool home
void mc_go_home();

// Check motion control status. result == 0: the system is idle. result > 0: the system is busy,
// result < 0: the system is in an error state.
int mc_status();

#endif
