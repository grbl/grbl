/*
  motion_plan.c - buffers movement commands and manages the acceleration profile plan
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

#include <inttypes.h>
#include <math.h>       

#include "motion_plan.h"
#include "nuts_bolts.h"
#include "stepper.h"

struct Block block_buffer[BLOCK_BUFFER_SIZE]; // A ring buffer for motion instructions
volatile int block_buffer_head = 0;           // Index of the next block to be pushed
volatile int block_buffer_tail = 0;           // Index of the block to process now

inline uint32_t estimate_acceleration_distance(int32_t current_rate, int32_t target_rate, int32_t acceleration) {
  return((target_rate*target_rate-current_rate*current_rate)/(2*acceleration));
}

inline uint32_t estimate_acceleration_ticks(int32_t start_rate, int32_t acceleration_per_tick, int32_t step_events) {
  return(
    round(
      (sqrt(2*acceleration_per_tick*step_events+(start_rate*start_rate))-start_rate)/
      acceleration_per_tick));
}

// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
// In practice both factors must be in the range 0 ... 1.0
void calculate_trapezoid_for_block(struct Block *block, double entry_factor, double exit_factor) {
  block->initial_rate = max(round(block->nominal_rate*entry_factor),MINIMAL_STEP_RATE);
  int32_t final_rate = max(round(block->nominal_rate*entry_factor),MINIMAL_STEP_RATE);
  int32_t acceleration_per_second = block->rate_delta*ACCELERATION_TICKS_PER_SECOND;
  int32_t acceleration_steps = 
    estimate_acceleration_distance(block->initial_rate, block->nominal_rate, acceleration_per_second);
  int32_t decelleration_steps = 
    estimate_acceleration_distance(block->nominal_rate, final_rate, -acceleration_per_second);
  // Check if the acceleration and decelleration periods overlap. In that case nominal_speed will
  // never be reached but that's okay. Just truncate both periods proportionally so that they
  // fit within the allotted step events.
  int32_t plateau_steps = block->step_event_count-acceleration_steps-decelleration_steps;
  if (plateau_steps < 0) {
    int32_t half_overlap_region = fabs(plateau_steps)/2;
    plateau_steps = 0;
    acceleration_steps = max(acceleration_steps-half_overlap_region,0);
    decelleration_steps = max(decelleration_steps-half_overlap_region,0);
  }
  block->accelerate_ticks = estimate_acceleration_ticks(block->initial_rate, block->rate_delta, acceleration_steps);
  if (plateau_steps) {
    block->plateau_ticks = round(1.0*plateau_steps/(block->nominal_rate*ACCELERATION_TICKS_PER_SECOND));
  } else {
    block->plateau_ticks = 0;
  }
}
