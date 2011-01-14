/*
  motion_plan.h - buffers movement commands and manages the acceleration profile plan
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

#ifndef motion_plan_h
#define motion_plan_h
                 
#include <inttypes.h>

// Pick a suitable block-buffer size
#ifdef __AVR_ATmega328P__
#define BLOCK_BUFFER_SIZE 20   // Atmega 328 has one full kilobyte of extra RAM!
#else
#define BLOCK_BUFFER_SIZE 5
#endif

// This struct is used when buffering the setup for each linear movement
// "nominal" values are as specified in the source g-code and may never
// actually be reached if acceleration management is active.
struct Block {
  uint32_t steps_x, steps_y, steps_z; // Step count along each axis
  uint8_t  direction_bits;            // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  int32_t  step_event_count;          // The number of step events required to complete this block
  uint32_t nominal_rate;              // The nominal step rate for this block in step_events/minute
  // Values used for acceleration management
  double  speed_x, speed_y, speed_z;   // Nominal mm/minute for each axis
  uint32_t initial_rate;              // The jerk-adjusted step rate at start of block
  int16_t rate_delta;                 // The steps/minute to add or subtract when changing speed (must be positive)
  uint16_t accelerate_ticks;          // The number of acceleration-ticks to accelerate 
  uint16_t plateau_ticks;             // The number of acceleration-ticks to maintain top speed
};
      
extern struct Block block_buffer[BLOCK_BUFFER_SIZE]; // A ring buffer for motion instructions
extern volatile int block_buffer_head;           // Index of the next block to be pushed
extern volatile int block_buffer_tail;           // Index of the block to process now

// Initialize the motion plan subsystem      
void mp_init();

// Add a new linear movement to the buffer. steps_x, _y and _z is the signed, relative motion in 
// steps. Microseconds specify how many microseconds the move should take to perform. To aid acceleration
// calculation the caller must also provide the physical length of the line in millimeters.
// Do not call directly unless you are writing a motor driver. In current iteration this is called by 
// st_buffer_line which also wakes up the stepper subsystem.
void mp_buffer_line(int32_t steps_x, int32_t steps_y, int32_t steps_z, uint32_t microseconds, double millimeters);

// Enables acceleration-management for upcoming blocks
void mp_enable_acceleration_management();
// Disables acceleration-management for upcoming blocks
void mp_disable_acceleration_management();


#endif