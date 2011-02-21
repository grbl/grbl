/*
  planner.c - buffers movement commands and manages the acceleration profile plan
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

/* The ring buffer implementation gleaned from the wiring_serial library by David A. Mellis. */

#include <inttypes.h>
#include <math.h>       
#include <stdlib.h>

#include "planner.h"
#include "nuts_bolts.h"
#include "stepper.h"
#include "settings.h"
#include "config.h"
#include "wiring_serial.h"

// The number of linear motions that can be in the plan at any give time
#ifdef __AVR_ATmega328P__
#define BLOCK_BUFFER_SIZE 16
#else
#define BLOCK_BUFFER_SIZE 5
#endif

static block_t block_buffer[BLOCK_BUFFER_SIZE];  // A ring buffer for motion instructions
static volatile uint8_t block_buffer_head;       // Index of the next block to be pushed
static volatile uint8_t block_buffer_tail;       // Index of the block to process now

// The current position of the tool in absolute steps
static int32_t position[3];   

static uint8_t acceleration_manager_enabled;   // Acceleration management active?

#define ONE_MINUTE_OF_MICROSECONDS 60000000.0

// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the 
// given acceleration:
static double estimate_acceleration_distance(double initial_rate, double target_rate, double acceleration) {
  return(
    (target_rate*target_rate-initial_rate*initial_rate)/
    (2L*acceleration)
  );
}

/*                        + <- some maximum rate we don't care about
                         /|\
                        / | \                    
                       /  |  + <- final_rate     
                      /   |  |                   
     initial_rate -> +----+--+                   
                          ^  ^                   
                          |  |                   
      intersection_distance  distance                                                                           */


// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

static double intersection_distance(double initial_rate, double final_rate, double acceleration, double distance) {
  return(
    (2*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/
    (4*acceleration)
  );
}

/*                                                                              
                                     +--------+   <- nominal_rate
                                    /          \                                
    nominal_rate*entry_factor ->   +            \                               
                                   |             + <- nominal_rate*exit_factor  
                                   +-------------+                              
                                       time -->                                 
*/                                                                              

// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
// The factors represent a factor of braking and must be in the range 0.0-1.0.

static void calculate_trapezoid_for_block(block_t *block, double entry_factor, double exit_factor) {
  block->initial_rate = ceil(block->nominal_rate*entry_factor);
  block->final_rate = ceil(block->nominal_rate*exit_factor);
  int32_t acceleration_per_minute = block->rate_delta*ACCELERATION_TICKS_PER_SECOND*60.0;
  int32_t accelerate_steps = 
    ceil(estimate_acceleration_distance(block->initial_rate, block->nominal_rate, acceleration_per_minute));
  int32_t decelerate_steps = 
    floor(estimate_acceleration_distance(block->nominal_rate, block->final_rate, -acceleration_per_minute));

  // Calculate the size of Plateau of Nominal Rate. 
  int32_t plateau_steps = block->step_event_count-accelerate_steps-decelerate_steps;
  
  // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
  // have to use intersection_distance() to calculate when to abort acceleration and start braking 
  // in order to reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0) {  
    accelerate_steps = ceil(
      intersection_distance(block->initial_rate, block->final_rate, acceleration_per_minute, block->step_event_count));
    plateau_steps = 0;
  }  
  
  block->accelerate_until = accelerate_steps;
  block->decelerate_after = accelerate_steps+plateau_steps;
}                    

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the 
// acceleration within the allotted distance.
static double max_allowable_speed(double acceleration, double target_velocity, double distance) {
  return(
    sqrt(target_velocity*target_velocity-2*acceleration*60*60*distance)
  );
}

// "Junction jerk" in this context is the immediate change in speed at the junction of two blocks.
// This method will calculate the junction jerk as the euclidean distance between the nominal 
// velocities of the respective blocks.
static double junction_jerk(block_t *before, block_t *after) {
  return(sqrt(
    pow(before->speed_x-after->speed_x, 2)+
    pow(before->speed_y-after->speed_y, 2)+
    pow(before->speed_z-after->speed_z, 2))
  );
}

// Calculate a braking factor to reach baseline speed which is max_jerk/2, e.g. the 
// speed under which you cannot exceed max_jerk no matter what you do.
static double factor_for_safe_speed(block_t *block) {
  if(settings.max_jerk < block->nominal_speed) {
    return(settings.max_jerk/block->nominal_speed);  
  } else {
    return(1.0);
  }
}

// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
static void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  if(!current) { return; }

  double entry_factor = 1.0;
  double exit_factor;
  if (next) {
    exit_factor = next->entry_factor;
  } else {
    exit_factor = factor_for_safe_speed(current);
  }
  
  // Calculate the entry_factor for the current block. 
  if (previous) {
    // Reduce speed so that junction_jerk is within the maximum allowed
    double jerk = junction_jerk(previous, current);
    if (jerk > settings.max_jerk) {
      entry_factor = (settings.max_jerk/jerk);
    } 
    // If the required deceleration across the block is too rapid, reduce the entry_factor accordingly.
    if (entry_factor > exit_factor) {
      double max_entry_speed = max_allowable_speed(-settings.acceleration,current->nominal_speed*exit_factor, 
        current->millimeters);
      double max_entry_factor = max_entry_speed/current->nominal_speed;
      if (max_entry_factor < entry_factor) {
        entry_factor = max_entry_factor;
      }
    }    
  } else {
    entry_factor = factor_for_safe_speed(current);
  }
    
  // Store result
  current->entry_factor = entry_factor;
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the reverse pass.
static void planner_reverse_pass() {
  auto int8_t block_index = block_buffer_head;
  block_t *block[3] = {NULL, NULL, NULL};
  while(block_index != block_buffer_tail) {    
    block_index--;
    if(block_index < 0) {
      block_index = BLOCK_BUFFER_SIZE-1;
    }
    block[2]= block[1];
    block[1]= block[0];
    block[0] = &block_buffer[block_index];
    planner_reverse_pass_kernel(block[0], block[1], block[2]);
  }
  planner_reverse_pass_kernel(NULL, block[0], block[1]);
}

// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
static void planner_forward_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  if(!current) { return; }
  // If the previous block is an acceleration block, but it is not long enough to 
  // complete the full speed change within the block, we need to adjust out entry
  // speed accordingly. Remember current->entry_factor equals the exit factor of 
  // the previous block.
  if(previous->entry_factor < current->entry_factor) {
    double max_entry_speed = max_allowable_speed(-settings.acceleration,
      current->nominal_speed*previous->entry_factor, previous->millimeters);
    double max_entry_factor = max_entry_speed/current->nominal_speed;
    if (max_entry_factor < current->entry_factor) {
      current->entry_factor = max_entry_factor;
    }
  }
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the forward pass.
static void planner_forward_pass() {
  int8_t block_index = block_buffer_tail;
  block_t *block[3] = {NULL, NULL, NULL};
  
  while(block_index != block_buffer_head) {
    block[0] = block[1];
    block[1] = block[2];
    block[2] = &block_buffer[block_index];
    planner_forward_pass_kernel(block[0],block[1],block[2]);
    block_index = (block_index+1) % BLOCK_BUFFER_SIZE;
  }
  planner_forward_pass_kernel(block[1], block[2], NULL);
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the 
// entry_factor for each junction. Must be called by planner_recalculate() after 
// updating the blocks.
static void planner_recalculate_trapezoids() {
  int8_t block_index = block_buffer_tail;
  block_t *current;
  block_t *next = NULL;
  
  while(block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    if (current) {
      calculate_trapezoid_for_block(current, current->entry_factor, next->entry_factor);      
    }
    block_index = (block_index+1) % BLOCK_BUFFER_SIZE;
  }
  calculate_trapezoid_for_block(next, next->entry_factor, factor_for_safe_speed(next));
}

// Recalculates the motion plan according to the following algorithm:
//
//   1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_factor) 
//      so that:
//     a. The junction jerk is within the set limit
//     b. No speed reduction within one block requires faster deceleration than the one, true constant 
//        acceleration.
//   2. Go over every block in chronological order and dial down junction speed reduction values if 
//     a. The speed increase within one block would require faster accelleration than the one, true 
//        constant acceleration.
//
// When these stages are complete all blocks have an entry_factor that will allow all speed changes to 
// be performed using only the one, true constant acceleration, and where no junction jerk is jerkier than 
// the set limit. Finally it will:
//
//   3. Recalculate trapezoids for all blocks.

static void planner_recalculate() {     
  planner_reverse_pass();
  planner_forward_pass();
  planner_recalculate_trapezoids();
}

void plan_init() {
  block_buffer_head = 0;
  block_buffer_tail = 0;
  plan_set_acceleration_manager_enabled(true);
  clear_vector(position);
}

void plan_set_acceleration_manager_enabled(int enabled) {
  if ((!!acceleration_manager_enabled) != (!!enabled)) {
    st_synchronize();
    acceleration_manager_enabled = !!enabled;
  }
}

int plan_is_acceleration_manager_enabled() {
  return(acceleration_manager_enabled);
}

void plan_discard_current_block() {
  if (block_buffer_head != block_buffer_tail) {
    block_buffer_tail = (block_buffer_tail + 1) % BLOCK_BUFFER_SIZE;  
  }
}

block_t *plan_get_current_block() {
  if (block_buffer_head == block_buffer_tail) { return(NULL); }
  return(&block_buffer[block_buffer_tail]);
}

// Add a new linear movement to the buffer. steps_x, _y and _z is the absolute position in 
// mm. Microseconds specify how many microseconds the move should take to perform. To aid acceleration
// calculation the caller must also provide the physical length of the line in millimeters.
void plan_buffer_line(double x, double y, double z, double feed_rate, int invert_feed_rate) {
  // The target position of the tool in absolute steps
  
  // Calculate target position in absolute steps
  int32_t target[3];
  target[X_AXIS] = lround(x*settings.steps_per_mm[X_AXIS]);
  target[Y_AXIS] = lround(y*settings.steps_per_mm[Y_AXIS]);
  target[Z_AXIS] = lround(z*settings.steps_per_mm[Z_AXIS]);     
  
  // Calculate the buffer head after we push this byte
	int next_buffer_head = (block_buffer_head + 1) % BLOCK_BUFFER_SIZE;	
	// If the buffer is full: good! That means we are well ahead of the robot. 
	// Rest here until there is room in the buffer.
  while(block_buffer_tail == next_buffer_head) { sleep_mode(); }
  // Prepare to set up new block
  block_t *block = &block_buffer[block_buffer_head];
  // Number of steps for each axis
  block->steps_x = labs(target[X_AXIS]-position[X_AXIS]);
  block->steps_y = labs(target[Y_AXIS]-position[Y_AXIS]);
  block->steps_z = labs(target[Z_AXIS]-position[Z_AXIS]);
  block->step_event_count = max(block->steps_x, max(block->steps_y, block->steps_z));
  // Bail if this is a zero-length block
  if (block->step_event_count == 0) { return; };
  
  double delta_x_mm = (target[X_AXIS]-position[X_AXIS])/settings.steps_per_mm[X_AXIS];
  double delta_y_mm = (target[Y_AXIS]-position[Y_AXIS])/settings.steps_per_mm[Y_AXIS];
  double delta_z_mm = (target[Z_AXIS]-position[Z_AXIS])/settings.steps_per_mm[Z_AXIS];
  block->millimeters = sqrt(square(delta_x_mm) + square(delta_y_mm) + square(delta_z_mm));
	
  
  uint32_t microseconds;
  if (!invert_feed_rate) {
    microseconds = lround((block->millimeters/feed_rate)*1000000);
  } else {
    microseconds = lround(ONE_MINUTE_OF_MICROSECONDS/feed_rate);
  }
  
  // Calculate speed in mm/minute for each axis
  double multiplier = 60.0*1000000.0/microseconds;
  block->speed_x = delta_x_mm * multiplier;
  block->speed_y = delta_y_mm * multiplier;
  block->speed_z = delta_z_mm * multiplier; 
  block->nominal_speed = block->millimeters * multiplier;
  block->nominal_rate = ceil(block->step_event_count * multiplier);  
  block->entry_factor = 0.0;
  
  // Compute the acceleration rate for the trapezoid generator. Depending on the slope of the line
  // average travel per step event changes. For a line along one axis the travel per step event
  // is equal to the travel/step in the particular axis. For a 45 degree line the steppers of both
  // axes might step for every step event. Travel per step event is then sqrt(travel_x^2+travel_y^2).
  // To generate trapezoids with contant acceleration between blocks the rate_delta must be computed 
  // specifically for each line to compensate for this phenomenon:
  double travel_per_step = block->millimeters/block->step_event_count;
  block->rate_delta = ceil(
    ((settings.acceleration*60.0)/(ACCELERATION_TICKS_PER_SECOND))/ // acceleration mm/sec/sec per acceleration_tick
    travel_per_step);                                               // convert to: acceleration steps/min/acceleration_tick    
  if (acceleration_manager_enabled) {
    // compute a preliminary conservative acceleration trapezoid
    double safe_speed_factor = factor_for_safe_speed(block);
    calculate_trapezoid_for_block(block, safe_speed_factor, safe_speed_factor); 
  } else {
    block->initial_rate = block->nominal_rate;
    block->final_rate = block->nominal_rate;
    block->accelerate_until = 0;
    block->decelerate_after = block->step_event_count;
    block->rate_delta = 0;
  }
  
  // Compute direction bits for this block
  block->direction_bits = 0;
  if (target[X_AXIS] < position[X_AXIS]) { block->direction_bits |= (1<<X_DIRECTION_BIT); }
  if (target[Y_AXIS] < position[Y_AXIS]) { block->direction_bits |= (1<<Y_DIRECTION_BIT); }
  if (target[Z_AXIS] < position[Z_AXIS]) { block->direction_bits |= (1<<Z_DIRECTION_BIT); }
  
  // Move buffer head
  block_buffer_head = next_buffer_head;     
  // Update position 
  memcpy(position, target, sizeof(target)); // position[] = target[]
  
  if (acceleration_manager_enabled) { planner_recalculate(); }  
  st_wake_up();
}

// Reset the planner position vector
void plan_set_current_position(double x, double y, double z) {
  position[X_AXIS] = lround(x*settings.steps_per_mm[X_AXIS]);
  position[Y_AXIS] = lround(y*settings.steps_per_mm[Y_AXIS]);
  position[Z_AXIS] = lround(z*settings.steps_per_mm[Z_AXIS]);     
}
