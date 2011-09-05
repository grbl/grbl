/*
  planner.c - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Modifications Copyright (c) 2011 Sungeun Jeon
  
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

// The number of linear motions that can be in the plan at any give time
#ifdef __AVR_ATmega328P__
#define BLOCK_BUFFER_SIZE 16
#else
#define BLOCK_BUFFER_SIZE 5
#endif

static block_t block_buffer[BLOCK_BUFFER_SIZE];  // A ring buffer for motion instructions
static volatile uint8_t block_buffer_head;       // Index of the next block to be pushed
static volatile uint8_t block_buffer_tail;       // Index of the block to process now

static int32_t position[3];             // The current position of the tool in absolute steps
static double previous_unit_vec[3];     // Unit vector of previous path line segment
static double previous_nominal_speed;   // Nominal speed of previous path line segment

static uint8_t acceleration_manager_enabled;   // Acceleration management active?

#define ONE_MINUTE_OF_MICROSECONDS 60000000.0


// Returns the index of the next block in the ring buffer
static int8_t next_block_index(int8_t block_index) {
  return( (block_index + 1) % BLOCK_BUFFER_SIZE );
}


// Returns the index of the previous block in the ring buffer
static int8_t prev_block_index(int8_t block_index) {
  block_index--;
  if (block_index < 0) { block_index = BLOCK_BUFFER_SIZE-1; }
  return(block_index);
}


// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the 
// given acceleration:
static double estimate_acceleration_distance(double initial_rate, double target_rate, double acceleration) {
  return( (target_rate*target_rate-initial_rate*initial_rate)/(2*acceleration) );
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
  return( (2*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/(4*acceleration) );
}

            
// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the 
// acceleration within the allotted distance.
static double max_allowable_speed(double acceleration, double target_velocity, double distance) {
  return( sqrt(target_velocity*target_velocity-2*acceleration*60*60*distance) );
}


// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
static void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  if (!current) { return; }
  
  double entry_speed = current->max_entry_speed; // Re-write to ensure at max possible speed
  double exit_speed;
  if (next) {
    exit_speed = next->entry_speed;
  } else {
    exit_speed = 0.0; // Assume last block has zero exit velocity
  }
  // If the required deceleration across the block is too rapid, reduce the entry_speed accordingly.
  if (entry_speed > exit_speed) {
    entry_speed = 
      min(max_allowable_speed(-settings.acceleration,exit_speed,current->millimeters),entry_speed);
  }    
  current->entry_speed = entry_speed;
}


// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the reverse pass.
static void planner_reverse_pass() {
  auto int8_t block_index = block_buffer_head;
  block_t *block[3] = {NULL, NULL, NULL};
  while(block_index != block_buffer_tail) {    
    block_index = prev_block_index( block_index );
    block[2]= block[1];
    block[1]= block[0];
    block[0] = &block_buffer[block_index];
    planner_reverse_pass_kernel(block[0], block[1], block[2]);
  }
  // Skip buffer tail to prevent over-writing the initial entry speed.
}


// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
static void planner_forward_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  if(!current) { return; }
  // If the previous block is an acceleration block, but it is not long enough to 
  // complete the full speed change within the block, we need to adjust the entry
  // speed accordingly.
  if(previous) {
    if (previous->entry_speed < current->entry_speed) {
      current->entry_speed = min( min( current->entry_speed, current->max_entry_speed ), 
        max_allowable_speed(-settings.acceleration,previous->entry_speed,previous->millimeters) );
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
    block_index = next_block_index( block_index );
  }
  planner_forward_pass_kernel(block[1], block[2], NULL);
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


// Recalculates the trapezoid speed profiles for all blocks in the plan according to the 
// entry_speed for each junction. Must be called by planner_recalculate() after 
// updating the blocks.
static void planner_recalculate_trapezoids() {
  int8_t block_index = block_buffer_tail;
  block_t *current;
  block_t *next = NULL;

  while(block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    if (current) {
      // Compute entry and exit factors for trapezoid calculations
      double entry_factor = current->entry_speed/current->nominal_speed;
      double exit_factor = next->entry_speed/current->nominal_speed;
      calculate_trapezoid_for_block(current, entry_factor, exit_factor);      
    }
    block_index = next_block_index( block_index );
  }
  calculate_trapezoid_for_block(next, next->entry_speed, 0.0); // Last block
}

// Recalculates the motion plan according to the following algorithm:
//
//   1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_speed) 
//      so that:
//     a. The maximum junction speed is within the set limit
//     b. No speed reduction within one block requires faster deceleration than the one, true constant 
//        acceleration.
//   2. Go over every block in chronological order and dial down junction speed values if 
//     a. The speed increase within one block would require faster acceleration than the one, true 
//        constant acceleration.
//
// When these stages are complete all blocks have an entry speed that will allow all speed changes to 
// be performed using only the one, true constant acceleration, and where no junction speed is greater
// than the set limit. Finally it will:
//
//   3. Recalculate trapezoids for all blocks using the recently updated junction speeds.

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
  clear_vector_double(previous_unit_vec);
  previous_nominal_speed = 0.0;
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
    block_buffer_tail = next_block_index( block_buffer_tail );
  }
}

block_t *plan_get_current_block() {
  if (block_buffer_head == block_buffer_tail) { return(NULL); }
  return(&block_buffer[block_buffer_tail]);
}


// Add a new linear movement to the buffer. x, y and z is the signed, absolute target position in 
// millimaters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
// rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
void plan_buffer_line(double x, double y, double z, double feed_rate, int invert_feed_rate) {
  // The target position of the tool in absolute steps
  
  // Calculate target position in absolute steps
  int32_t target[3];
  target[X_AXIS] = lround(x*settings.steps_per_mm[X_AXIS]);
  target[Y_AXIS] = lround(y*settings.steps_per_mm[Y_AXIS]);
  target[Z_AXIS] = lround(z*settings.steps_per_mm[Z_AXIS]);     
  
  // Calculate the buffer head after we push this byte
  int next_buffer_head = next_block_index( block_buffer_head );	
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
  
  // Compute path vector in terms of quantized step target and current positions
  double delta_mm[3];
  delta_mm[X_AXIS] = (target[X_AXIS]-position[X_AXIS])/settings.steps_per_mm[X_AXIS];
  delta_mm[Y_AXIS] = (target[Y_AXIS]-position[Y_AXIS])/settings.steps_per_mm[Y_AXIS];
  delta_mm[Z_AXIS] = (target[Z_AXIS]-position[Z_AXIS])/settings.steps_per_mm[Z_AXIS];
  block->millimeters = sqrt(square(delta_mm[X_AXIS]) + square(delta_mm[Y_AXIS]) + 
                            square(delta_mm[Z_AXIS]));
	
  uint32_t microseconds;
  if (!invert_feed_rate) {
    microseconds = lround((block->millimeters/feed_rate)*1000000);
  } else {
    microseconds = lround(ONE_MINUTE_OF_MICROSECONDS/feed_rate);
  }
  
  // Calculate speed in mm/minute for each axis
  double multiplier = 60.0*1000000.0/microseconds;
  block->speed_x = delta_mm[X_AXIS] * multiplier;
  block->speed_y = delta_mm[Y_AXIS] * multiplier;
  block->speed_z = delta_mm[Z_AXIS] * multiplier; 
  block->nominal_speed = block->millimeters * multiplier;
  block->nominal_rate = ceil(block->step_event_count * multiplier);  
  
  // This is a temporary fix to avoid a situation where very low nominal_speeds would be rounded 
  // down to zero and cause a division by zero. TODO: Grbl deserves a less patchy fix for this problem
  if (block->nominal_speed < 60.0) { block->nominal_speed = 60.0; }

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

    // Compute path unit vector                            
    double unit_vec[3];                          
    unit_vec[X_AXIS] = delta_mm[X_AXIS]/block->millimeters;
    unit_vec[Y_AXIS] = delta_mm[Y_AXIS]/block->millimeters;
    unit_vec[Z_AXIS] = delta_mm[Z_AXIS]/block->millimeters;  
  
    // Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
    // Let a circle be tangent to both previous and current path line segments, where the junction 
    // deviation is defined as the distance from the junction to the edge of the circle. The
    // circular segment joining the two paths represents the path of centripetal acceleration.
    // Solve for max velocity based on max acceleration about the radius of the circle, defined 
    // indirectly by junction deviation, which may be also viewed as path width or max_jerk.
    double vmax_junction = 0.0; // Set default zero max junction speed
    
    // Use default for first block or when planner is reset by previous_nominal_speed = 0.0
    if ((block_buffer_head != block_buffer_tail) && (previous_nominal_speed > 0.0)) {
      // Compute cosine of angle between previous and current path
      double cos_theta = ( -previous_unit_vec[X_AXIS] * unit_vec[X_AXIS] + 
                           -previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS] + 
                           -previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS] );
                           
      // Avoid divide by zero for straight junctions at 180 degrees. Limit to min() of nominal speeds.
      vmax_junction = min(previous_nominal_speed,block->nominal_speed);
      if (cos_theta > -1.0) {
        // Compute maximum junction velocity based on maximum acceleration and junction deviation
        double sin_theta_d2 = sqrt(0.5*(1.0-cos_theta)); // Trig half angle identity
        vmax_junction = max(0.0, min(vmax_junction,
          sqrt(settings.acceleration*60*60 * settings.junction_deviation * sin_theta_d2/(1.0-sin_theta_d2)) ) );
      }
    }
    
    block->max_entry_speed = vmax_junction;
    block->entry_speed = vmax_junction;
  
    // Update previous path unit_vector and nominal speed
    memcpy(previous_unit_vec, unit_vec, sizeof(unit_vec)); // previous_unit_vec[] = unit_vec[]
    previous_nominal_speed = block->nominal_speed;

  } else {
    // Set at nominal rates only for disabled acceleration planner
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

// Reset the planner position vector and planner speed
void plan_set_current_position(double x, double y, double z) {
  position[X_AXIS] = lround(x*settings.steps_per_mm[X_AXIS]);
  position[Y_AXIS] = lround(y*settings.steps_per_mm[Y_AXIS]);
  position[Z_AXIS] = lround(z*settings.steps_per_mm[Z_AXIS]);     
  previous_nominal_speed = 0.0;
}
