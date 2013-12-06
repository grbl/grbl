/*
  planner.h - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

  Copyright (c) 2011-2013 Sungeun K. Jeon 
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

#ifndef planner_h
#define planner_h
#include "nuts_bolts.h"

// The number of linear motions that can be in the plan at any give time
#ifndef BLOCK_BUFFER_SIZE
  #define BLOCK_BUFFER_SIZE 17
#endif

// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in 
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {

  // Fields used by the bresenham algorithm for tracing the line
  uint8_t  direction_bits;            // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  uint32_t steps[N_AXIS];             // Step count along each axis
  int32_t  step_event_count;          // The number of step events required to complete this block

  // Fields used by the motion planner to manage acceleration
  float nominal_speed_sqr;           // Axis-limit adjusted nominal speed for this block in (mm/min)^2
  float entry_speed_sqr;             // Entry speed at previous-current block junction in (mm/min)^2
  float max_entry_speed_sqr;         // Maximum allowable junction entry speed in (mm/min)^2
  float millimeters;                 // The total travel of this block in mm
  float acceleration;                // Axes-limit adjusted line acceleration in mm/min^2

  // Settings for the trapezoid generator
  uint32_t initial_rate;              // The step rate at start of block  
  int32_t rate_delta;                 // The steps/minute to add or subtract when changing speed (must be positive)
  uint32_t decelerate_after;          // The index of the step event on which to start decelerating
  uint32_t nominal_rate;              // The nominal step rate for this block in step_events/minute
  uint32_t d_next;                    // Scaled distance to next step

} block_t;
      
// Initialize the motion plan subsystem
void plan_init();

// Add a new linear movement to the buffer. target[N_AXIS] is the signed, absolute target position 
// in millimeters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
// rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
void plan_buffer_line(float *target, float feed_rate, uint8_t invert_feed_rate);

// Called when the current block is no longer needed. Discards the block and makes the memory
// availible for new blocks.
void plan_discard_current_block();

// Gets the current block. Returns NULL if buffer empty
block_t *plan_get_current_block();

// Reset the planner position vector (in steps)
void plan_sync_position();

// Reinitialize plan with a partially completed block
void plan_cycle_reinitialize(int32_t step_events_remaining);

// Returns the status of the block ring buffer. True, if buffer is full.
uint8_t plan_check_full_buffer();

// Block until all buffered steps are executed
void plan_synchronize();

#endif
