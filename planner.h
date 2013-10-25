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
  #define BLOCK_BUFFER_SIZE 18
#endif

// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in 
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {

  // Fields used by the bresenham algorithm for tracing the line
  // NOTE: Do not change any of these values once set. The stepper algorithm uses them to execute the block correctly.
  uint8_t direction_bits;            // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  int32_t steps[N_AXIS];             // Step count along each axis
  int32_t step_event_count;          // The maximum step axis count and number of steps required to complete this block. 

  // Fields used by the motion planner to manage acceleration
  float entry_speed_sqr;             // The current planned entry speed at block junction in (mm/min)^2
  float max_entry_speed_sqr;         // Maximum allowable entry speed based on the minimum of junction limit and 
                                     //   neighboring nominal speeds with overrides in (mm/min)^2
  float max_junction_speed_sqr;      // Junction entry speed limit based on direction vectors in (mm/min)^2
  float nominal_speed_sqr;           // Axis-limit adjusted nominal speed for this block in (mm/min)^2
  float acceleration;                // Axis-limit adjusted line acceleration in mm/min^2
  float millimeters;                 // The remaining distance for this block to be executed in mm
  
} plan_block_t;
      
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
plan_block_t *plan_get_current_block();

uint8_t plan_next_block_index(uint8_t block_index);

plan_block_t *plan_get_block_by_index(uint8_t block_index);

float plan_calculate_velocity_profile(uint8_t block_index);

// void plan_update_partial_block(uint8_t block_index, float millimeters_remaining, uint8_t is_decelerating);

// Reset the planner position vector (in steps)
void plan_sync_position();

// Reinitialize plan with a partially completed block
void plan_cycle_reinitialize(int32_t step_events_remaining);

// Returns the status of the block ring buffer. True, if buffer is full.
uint8_t plan_check_full_buffer();

// Block until all buffered steps are executed
void plan_synchronize();

#endif
