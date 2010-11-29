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

// Initializes the motion_control subsystem resources
void mc_init();

// Execute linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
// unless invert_feed_rate is true. Then the feed_rate means that the motion should be completed in
// (1 minute)/feed_rate time.
void mc_line(double x, double y, double z, float feed_rate, int invert_feed_rate, int32_t line_number);

void mc_reposition(double x, double y, double z, int32_t line_number);

// Execute an arc. theta == start angle, angular_travel == number of radians to go along the arc,
// positive angular_travel means clockwise, negative means counterclockwise. Radius == the radius of the
// circle in millimeters. axis_1 and axis_2 selects the circle plane in tool space. Stick the remaining
// axis in axis_l which will be the axis for linear travel if you are tracing a helical motion.
void mc_arc(double theta, double angular_travel, double radius, double linear_travel, 
            int axis_1, int axis_2, int axis_linear, 
            double target_x, double target_y, double target_z,
            double feed_rate, int invert_feed_rate, int32_t line_number);

void mc_continue_arc();

// Dwell for a couple of time units
void mc_dwell(uint32_t milliseconds, int32_t line_number);

// Send the tool home
void mc_go_home(int32_t line_number);

// Stop any current motions
void mc_stop();

// Stop executing the current motion and go to the next one:
//void mc_next(int line_number);

// Continue executing stopped command.
//void mc_continue(int line_number);

extern volatile char mc_running;
extern int32_t acting_line_number;

uint8_t mc_in_arc();


#endif
