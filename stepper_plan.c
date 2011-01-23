/*
  stepper_plan.c - buffers movement commands and manages the acceleration profile plan
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
#include <stdlib.h>

#include "stepper_plan.h"
#include "nuts_bolts.h"
#include "stepper.h"
#include "config.h"
#include "wiring_serial.h"

struct Block block_buffer[BLOCK_BUFFER_SIZE]; // A ring buffer for motion instructions
volatile int block_buffer_head;           // Index of the next block to be pushed
volatile int block_buffer_tail;           // Index of the block to process now
uint8_t acceleration_management;              // Acceleration management active?



// The distance it takes to accelerate from initial_rate to target_rate using the given acceleration
inline double estimate_acceleration_distance(double initial_rate, double target_rate, double acceleration) {
  return((target_rate*target_rate-initial_rate*initial_rate)/(2L*acceleration));
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

/*                        + <- some rate that the client must be certain will not exceed the maximum allowable
                         /|\
                        / | \                    
                       /  |  + <- final_rate     
                      /   |  |                   
     initial_rate -> +----+--+                   
                          ^  ^                   
                          |  |                   
      intersection_distance  distance                                                                           */

inline double intersection_distance(double initial_rate, double final_rate, double acceleration, double distance) {
  return((2*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/(4*acceleration));
}

// See bottom of this module for a comment outlining the reasoning behind the mathematics behind the
// preceding functions.

// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
// In practice both factors must be in the range 0 ... 1.0
void calculate_trapezoid_for_block(struct Block *block, double entry_factor, double exit_factor) {
  block->initial_rate = ceil(block->nominal_rate*entry_factor);
  int32_t final_rate = ceil(block->nominal_rate*entry_factor);
  int32_t acceleration_per_minute = block->rate_delta*ACCELERATION_TICKS_PER_SECOND*60.0;
  int32_t accelerate_steps = 
    ceil(estimate_acceleration_distance(block->initial_rate, block->nominal_rate, acceleration_per_minute));
  int32_t decelerate_steps = 
    estimate_acceleration_distance(block->nominal_rate, final_rate, -acceleration_per_minute);
  printString("ir="); printInteger(block->initial_rate); printString("\n\r");
  printString("nr="); printInteger(block->nominal_rate); printString("\n\r");
  printString("rd="); printInteger(block->rate_delta); printString("\n\r");
  printString("aps="); printInteger(acceleration_per_minute); printString("\n\r");
  printString("acs="); printInteger(accelerate_steps); printString("\n\r");
  printString("dcs="); printInteger(decelerate_steps); printString("\n\r");
  printString("ts="); printInteger(block->step_event_count); printString("\n\r");
  // Check if the acceleration and decelleration periods overlap. In that case nominal_speed will
  // never be reached but that's okay. Just truncate both periods proportionally so that they
  // fit within the allotted step events.
  int32_t plateau_steps = block->step_event_count-accelerate_steps-decelerate_steps;
  if (plateau_steps < 0) {  
    accelerate_steps = ceil(
      intersection_distance(block->initial_rate, final_rate, acceleration_per_minute, block->step_event_count));
    plateau_steps = 0;
    printString("No plateau, so: acs="); printInteger(accelerate_steps); printString("\n\r");
  }  
  block->accelerate_until = accelerate_steps;
  block->decelerate_after = accelerate_steps+plateau_steps;
}                    

inline double estimate_max_speed(double max_acceleration, double target_velocity, double distance) {
  return(sqrt(-2*max_acceleration*distance+target_velocity*target_velocity));
}

inline double estimate_jerk(struct Block *before, struct Block *after) {
  return(max(fabs(before->speed_x-after->speed_x),
    max(fabs(before->speed_y-after->speed_y), 
    fabs(before->speed_z-after->speed_z))));
}

// Builds plan for a single block provided. Returns TRUE if changes were made to this block
// that requires any earlier blocks to be recalculated too.
int8_t build_plan_for_single_block(struct Block *previous, struct Block *current, struct Block *next) {
  if(!current){return(TRUE);}
  double exit_factor;
  double entry_factor = 1.0;
  if (next) {
    exit_factor = next->entry_factor;
  } else {
    exit_factor = 0.0;
  }
  
  if (previous) {
    double jerk = estimate_jerk(previous, current);
    if (jerk > settings.max_jerk) {
      entry_factor = (settings.max_jerk/jerk);
    } 
    if (exit_factor<entry_factor) {
      double max_entry_speed = estimate_max_speed(-settings.acceleration,current->nominal_speed*exit_factor, 
        current->millimeters);
      double max_entry_factor = max_entry_speed/current->nominal_speed;
      if (max_entry_factor < entry_factor) {
        entry_factor = max_entry_factor;
      }
    }    
  } else {
    entry_factor = 0.0;
  }
  // Check if we made a difference for this block. If we didn't, the planner can call it quits
  // here. No need to process any earlier blocks.
  int8_t keep_going = (entry_factor > current->entry_factor ? TRUE : FALSE);  
  // Store result and recalculate trapezoid parameters
  current->entry_factor = entry_factor;
  calculate_trapezoid_for_block(current, entry_factor, exit_factor);
  return(keep_going);
}

void recalculate_plan() {
  int8_t block_index = block_buffer_head;
  struct Block *block[3] = {NULL, NULL, NULL};
  while(block_index != block_buffer_tail) {
    block[2]= block[1];
    block[1]= block[0];
    block[0] = &block_buffer[block_index];
    if (!build_plan_for_single_block(block[0], block[1], block[2])) {return;}
    block_index = (block_index-1) % BLOCK_BUFFER_SIZE;
  }
  if (block[1]) {
    calculate_trapezoid_for_block(block[0], block[0]->entry_factor, block[1]->entry_factor);
  }
}

void plan_enable_acceleration_management() {
  if (!acceleration_management) {
    st_synchronize();
    acceleration_management = TRUE;
  }
}

void plan_disable_acceleration_management() {
  if(acceleration_management) {
    st_synchronize();
    acceleration_management = FALSE;
  }
}

void plan_init() {
  block_buffer_head = 0;
  block_buffer_tail = 0;
  plan_enable_acceleration_management();
}

// Add a new linear movement to the buffer. steps_x, _y and _z is the signed, relative motion in 
// steps. Microseconds specify how many microseconds the move should take to perform. To aid acceleration
// calculation the caller must also provide the physical length of the line in millimeters.
void plan_buffer_line(int32_t steps_x, int32_t steps_y, int32_t steps_z, uint32_t microseconds, double millimeters) {
  // Calculate the buffer head after we push this byte
	int next_buffer_head = (block_buffer_head + 1) % BLOCK_BUFFER_SIZE;	
	// If the buffer is full: good! That means we are well ahead of the robot. 
	// Rest here until there is room in the buffer.
  while(block_buffer_tail == next_buffer_head) { sleep_mode(); }
  // Prepare to set up new block
  struct Block *block = &block_buffer[block_buffer_head];
  // Number of steps for each axis
  block->steps_x = labs(steps_x);
  block->steps_y = labs(steps_y);
  block->steps_z = labs(steps_z);   
  block->step_event_count = max(block->steps_x, max(block->steps_y, block->steps_z));
  // Bail if this is a zero-length block
  if (block->step_event_count == 0) { return; };
  // Calculate speed in mm/minute for each axis
  double multiplier = 60.0*1000000.0/microseconds;
  block->speed_x = block->steps_x*multiplier/settings.steps_per_mm[0];
  block->speed_y = block->steps_y*multiplier/settings.steps_per_mm[1];
  block->speed_z = block->steps_z*multiplier/settings.steps_per_mm[2];
  block->nominal_speed = millimeters*multiplier;
  block->nominal_rate = ceil(block->step_event_count*multiplier);  
  
  // Compute the acceleration rate for the trapezoid generator. Depending on the slope of the line
  // average travel per step event changes. For a line along one axis the travel per step event
  // is equal to the travel/step in the particular axis. For a 45 degree line the steppers of both
  // axes might step for every step event. Travel per step event is then sqrt(travel_x^2+travel_y^2).
  // To generate trapezoids with contant acceleration between blocks the rate_delta must be computed 
  // specifically for each line to compensate for this phenomenon:
  double travel_per_step = millimeters/block->step_event_count;
  printString("travel_per_step*10000=");
  printInteger(travel_per_step*10000);printString("\n\r");
  block->rate_delta = ceil(
    ((settings.acceleration*60.0)/(ACCELERATION_TICKS_PER_SECOND))/ // acceleration mm/sec/sec per acceleration_tick
    travel_per_step);                                           // convert to: acceleration steps/min/acceleration_tick    
  if (acceleration_management) {
    calculate_trapezoid_for_block(block,0,0);                     // compute a conservative acceleration trapezoid for now
  } else {
    block->accelerate_until = 0;
    block->decelerate_after = 0;
    block->rate_delta = 0;
  }
  
  // Compute direction bits for this block
  block->direction_bits = 0;
  if (steps_x < 0) { block->direction_bits |= (1<<X_DIRECTION_BIT); }
  if (steps_y < 0) { block->direction_bits |= (1<<Y_DIRECTION_BIT); }
  if (steps_z < 0) { block->direction_bits |= (1<<Z_DIRECTION_BIT); }
  // Move buffer head
  block_buffer_head = next_buffer_head;
}


/*  
  Mathematica reasoning behind the mathematics in this module:
  
  s == speed, a == acceleration, t == time, d == distance

  Basic definitions:

    Speed[s_, a_, t_] := s + (a*t) 
    Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]

  Distance to reach a specific speed with a constant acceleration:

    Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
      d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()

  Speed after a given distance of travel with constant acceleration:

    Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
      m -> Sqrt[2 a d + s^2]    

    DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]

  When to start braking (di) to reach a specified destionation speed (s2) after accelerating
  from initial speed s1 without ever stopping at a plateau:

    Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
      di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()

    IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
*/
                                                                                                            
