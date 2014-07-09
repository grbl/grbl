/*
  motion_control.h - high level interface for issuing motion commands
  Part of Grbl

  Copyright (c) 2011-2014 Sungeun K. Jeon
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

#define HOMING_CYCLE_LINE_NUMBER -1

// Execute linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
// unless invert_feed_rate is true. Then the feed_rate means that the motion should be completed in
// (1 minute)/feed_rate time.
#ifdef USE_LINE_NUMBERS
void mc_line(float *target, float feed_rate, uint8_t invert_feed_rate, int32_t line_number);
#else
void mc_line(float *target, float feed_rate, uint8_t invert_feed_rate);
#endif

// Execute an arc in offset mode format. position == current xyz, target == target xyz, 
// offset == offset from current xyz, axis_XXX defines circle plane in tool space, axis_linear is
// the direction of helical travel, radius == circle radius, isclockwise boolean. Used
// for vector transformation direction.
#ifdef USE_LINE_NUMBERS
void mc_arc(float *position, float *target, float *offset, float radius, float feed_rate, 
  uint8_t invert_feed_rate, uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, int32_t line_number);
#else
void mc_arc(float *position, float *target, float *offset, float radius, float feed_rate,
  uint8_t invert_feed_rate, uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear);
#endif
  
// Dwell for a specific number of seconds
void mc_dwell(float seconds);

// Perform homing cycle to locate machine zero. Requires limit switches.
void mc_homing_cycle();

// Perform tool length probe cycle. Requires probe switch.
#ifdef USE_LINE_NUMBERS
void mc_probe_cycle(float *target, float feed_rate, uint8_t invert_feed_rate, int32_t line_number);
#else
void mc_probe_cycle(float *target, float feed_rate, uint8_t invert_feed_rate);
#endif

// Performs system reset. If in motion state, kills all motion and sets system alarm.
void mc_reset();

#endif
