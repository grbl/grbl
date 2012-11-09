/*
  planner.h - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011-2012 Sungeun K. Jeon  

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
                 
// The number of linear motions that can be in the plan at any give time
#ifndef BLOCK_BUFFER_SIZE
  #define BLOCK_BUFFER_SIZE 18
#endif

// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in 
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {

  // Fields used by the bresenham algorithm for tracing the line
  uint8_t  direction_bits;            // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  uint32_t steps_x, steps_y, steps_z; // Step count along each axis
  int32_t  step_event_count;          // The number of step events required to complete this block

  // Fields used by the motion planner to manage acceleration
  float nominal_speed;               // The nominal speed for this block in mm/min  
  float entry_speed;                 // Entry speed at previous-current block junction in mm/min
  float max_entry_speed;             // Maximum allowable junction entry speed in mm/min
  float millimeters;                 // The total travel of this block in mm
  uint8_t recalculate_flag;           // Planner flag to recalculate trapezoids on entry junction
  uint8_t nominal_length_flag;        // Planner flag for nominal speed always reached

  // Settings for the trapezoid generator
  uint32_t initial_rate;              // The step rate at start of block  
  uint32_t final_rate;                // The step rate at end of block
  int32_t rate_delta;                 // The steps/minute to add or subtract when changing speed (must be positive)
  uint32_t accelerate_until;          // The index of the step event on which to stop acceleration
  uint32_t decelerate_after;          // The index of the step event on which to start decelerating
  uint32_t nominal_rate;              // The nominal step rate for this block in step_events/minute

} block_t;
      
// Initialize the motion plan subsystem      
void plan_init();

// Add a new linear movement to the buffer. x, y and z is the signed, absolute target position in 
// millimaters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
// rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
void plan_buffer_line(float x, float y, float z, float feed_rate, uint8_t invert_feed_rate);

// Called when the current block is no longer needed. Discards the block and makes the memory
// availible for new blocks.
void plan_discard_current_block();

// Gets the current block. Returns NULL if buffer empty
block_t *plan_get_current_block();

// Reset the planner position vector (in steps)
void plan_set_current_position(int32_t x, int32_t y, int32_t z);

// Reinitialize plan with a partially completed block
void plan_cycle_reinitialize(int32_t step_events_remaining);

// Reset buffer
void plan_reset_buffer();

// Returns the status of the block ring buffer. True, if buffer is full.
uint8_t plan_check_full_buffer();

// Block until all buffered steps are executed
void plan_synchronize();

#endif
