/*
  planner.c - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Modifications Copyright (c) 2011 Sungeun (Sonny) Jeon
  
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

            
// Calculates the square of the maximum allowable speed at this point when you must be able to reach 
// target_velocity using the acceleration within the allotted distance.
// NOTE: sqrt() removed for speed optimization. Related calculations in terms of square velocity.
static double max_allowable_speed_sqr(double acceleration, double target_velocity_sqr, double distance) {
  return( target_velocity_sqr-2*acceleration*60*60*distance );
}


// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
static void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  if (!current) { return; }
  
  double entry_speed_sqr = current->max_entry_speed_sqr; // Reset and check to ensure max possible speed
    
  // If nominal length true, nominal speed is guaranteed to be reached. No need to re-compute.
  // But, if forward planner changed entry speed, reset to max entry speed just to be sure.
  if (!current->nominal_length_flag) {
    if (next) {
      // If the required deceleration across the block is too rapid, reduce entry_speed_sqr accordingly.
      if (entry_speed_sqr > next->entry_speed_sqr) {
        entry_speed_sqr = min( entry_speed_sqr,
          max_allowable_speed_sqr(-settings.acceleration,next->entry_speed_sqr,current->millimeters));
      }
    } else { 
      // Assume last block has zero exit velocity.
      entry_speed_sqr = min( entry_speed_sqr,
          max_allowable_speed_sqr(-settings.acceleration,0.0,current->millimeters));
    }
  }

  // Check for junction speed change
  if (current->entry_speed_sqr != entry_speed_sqr) {
    current->entry_speed_sqr = entry_speed_sqr;
    current->recalculate_flag = true; // Note: Newest block already set to true
  }
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

  if(previous) {

    // If nominal length true, nominal speed is guaranteed to be reached. No need to recalculate.  
    if (!previous->nominal_length_flag) {
      // If the previous block is an acceleration block, but it is not long enough to 
      // complete the full speed change within the block, we need to adjust the entry
      // speed accordingly.
      if (previous->entry_speed_sqr < current->entry_speed_sqr) {
        double entry_speed_sqr = min( current->entry_speed_sqr, current->max_entry_speed_sqr );
        entry_speed_sqr = min( entry_speed_sqr,
          max_allowable_speed_sqr(-settings.acceleration,previous->entry_speed_sqr,previous->millimeters) );

        // Check for junction speed change
        if (current->entry_speed_sqr != entry_speed_sqr) {
          current->entry_speed_sqr = entry_speed_sqr;
          current->recalculate_flag = true;
        }
      }
      
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


/*                             STEPPER RATE DEFINITION                                              
                                     +--------+   <- nominal_rate
                                    /          \                                
    nominal_rate*entry_factor ->   +            \                               
                                   |             + <- nominal_rate*exit_factor  
                                   +-------------+                              
                                       time -->                                 
*/                                                                              
// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
// The factors represent a factor of braking and must be in the range 0.0-1.0.
// This converts the planner parameters to the data required by the stepper controller.
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

/*                            PLANNER SPEED DEFINITION                                              
                                     +--------+   <- current->nominal_speed
                                    /          \                                
         current->entry_speed ->   +            \                               
                                   |             + <- next->entry_speed
                                   +-------------+                              
                                       time -->                                 
*/                                                                              
// Recalculates the trapezoid speed profiles for flagged blocks in the plan according to the 
// entry_speed for each junction and the entry_speed of the next junction. Must be called by 
// planner_recalculate() after updating the blocks. Any recalulate flagged junction will
// compute the two adjacent trapezoids to the junction, since the junction speed corresponds 
// to exit speed and entry speed of one another.
static void planner_recalculate_trapezoids() {
  int8_t block_index = block_buffer_tail;
  block_t *current;
  block_t *next = NULL;

  while(block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    if (current) {
      // Recalculate if current block entry or exit junction speed has changed.
      if (current->recalculate_flag || next->recalculate_flag) {
        // Compute entry and exit factors for trapezoid calculations. 
        // NOTE: sqrt(square velocities) now performed only when required in trapezoid calculation.
        double entry_factor = sqrt( current->entry_speed_sqr ) / current->nominal_speed;
        double exit_factor = sqrt( next->entry_speed_sqr ) / current->nominal_speed;
        calculate_trapezoid_for_block(current, entry_factor, exit_factor);      
        current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
      }
    }
    block_index = next_block_index( block_index );
  }
  // Last/newest block in buffer. Exit speed is zero.
  calculate_trapezoid_for_block(next, sqrt( next->entry_speed_sqr ) / next->nominal_speed, 0.0);
  next->recalculate_flag = false;
}

// Recalculates the motion plan according to the following algorithm:
//
//   1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_speed) 
//      so that:
//     a. The junction speed is equal to or less than the maximum junction speed limit
//     b. No speed reduction within one block requires faster deceleration than the one, true constant 
//        acceleration.
//   2. Go over every block in chronological order and dial down junction speed values if 
//     a. The speed increase within one block would require faster acceleration than the one, true 
//        constant acceleration.
//
// When these stages are complete all blocks have an entry speed that will allow all speed changes to 
// be performed using only the one, true constant acceleration, and where no junction speed is greater
// than the max limit. Finally it will:
//
//   3. Recalculate trapezoids for all blocks using the recently updated junction speeds. Block trapezoids
//      with no updated junction speeds will not be recalculated and assumed ok as is.

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

void plan_set_acceleration_manager_enabled(uint8_t enabled) {
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
void plan_buffer_line(double x, double y, double z, double feed_rate, uint8_t invert_feed_rate) {
  
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

  // Compute direction bits for this block
  block->direction_bits = 0;
  if (target[X_AXIS] < position[X_AXIS]) { block->direction_bits |= (1<<X_DIRECTION_BIT); }
  if (target[Y_AXIS] < position[Y_AXIS]) { block->direction_bits |= (1<<Y_DIRECTION_BIT); }
  if (target[Z_AXIS] < position[Z_AXIS]) { block->direction_bits |= (1<<Z_DIRECTION_BIT); }
  
  // Number of steps for each axis
  block->steps_x = labs(target[X_AXIS]-position[X_AXIS]);
  block->steps_y = labs(target[Y_AXIS]-position[Y_AXIS]);
  block->steps_z = labs(target[Z_AXIS]-position[Z_AXIS]);
  block->step_event_count = max(block->steps_x, max(block->steps_y, block->steps_z));

  // Bail if this is a zero-length block
  if (block->step_event_count == 0) { return; };
  
  // Compute path vector in terms of absolute step target and current positions
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
  double step_per_travel = block->step_event_count/block->millimeters; // Compute inverse to remove divide
  block->rate_delta = step_per_travel * ceil(                        // convert to: acceleration steps/min/acceleration_tick    
    settings.acceleration*60.0 / ACCELERATION_TICKS_PER_SECOND );    // acceleration mm/sec/sec per acceleration_tick

  // Perform planner-enabled calculations
  if (acceleration_manager_enabled) {  
  
    // Compute path unit vector                            
    double unit_vec[3];
    double inv_millimeters = 1.0/block->millimeters;  // Inverse millimeters to remove multiple divides
    unit_vec[X_AXIS] = delta_mm[X_AXIS]*inv_millimeters;
    unit_vec[Y_AXIS] = delta_mm[Y_AXIS]*inv_millimeters;
    unit_vec[Z_AXIS] = delta_mm[Z_AXIS]*inv_millimeters;  
  
    // Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
    // Does not actually deviate from path, but used as a robust way to compute cornering speeds.
    // Let a circle be tangent to both previous and current path line segments, where the junction 
    // deviation is defined as the distance from the junction to the closest edge of the circle, 
    // colinear with the circle center. The circular segment joining the two paths represents the 
    // path of centripetal acceleration. Solve for max velocity based on max acceleration about the
    // radius of the circle, defined indirectly by junction deviation. This may be also viewed as 
    // path width or max_jerk in the previous grbl version.
    // NOTE: sqrt() removed for speed optimization. Related calculations in terms of square velocity.

    double vmax_junction_sqr = 0.0; // Set default zero max junction speed

    // Skip first block or when previous_nominal_speed is used as a flag for homing and offset cycles.
    if ((block_buffer_head != block_buffer_tail) && (previous_nominal_speed > 0.0)) {
      // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
      // NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
      double cos_theta = - previous_unit_vec[X_AXIS] * unit_vec[X_AXIS] 
                         - previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS] 
                         - previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS] ;
                           
      // Skip and use default zero max junction speed for 0 degree acute junction.
      if (cos_theta < 1.0) {
        vmax_junction_sqr = square( min(previous_nominal_speed,block->nominal_speed) );
        // Skip and avoid divide by zero for straight junctions at 180 degrees. Limit to min() of nominal speeds.
        if (cos_theta > -1.0) {
          // Compute maximum junction velocity based on maximum acceleration and junction deviation
          double sin_theta_d2 = sqrt(0.5*(1.0-cos_theta)); // Trig half angle identity. Always positive.
          vmax_junction_sqr = min(vmax_junction_sqr,
            settings.acceleration*60*60 * settings.junction_deviation * sin_theta_d2/(1.0-sin_theta_d2) );
        }
      }
    }
    block->max_entry_speed_sqr = vmax_junction_sqr;
    block->entry_speed_sqr = vmax_junction_sqr;

    // Initialize planner efficiency flags
    // Set flag if block will always reach nominal speed regardless of entry/exit speeds.
    if (block->nominal_speed <= sqrt(max_allowable_speed_sqr(-settings.acceleration,0.0,0.5*block->millimeters)) ) 
      { block->nominal_length_flag = true; }
    else { block->nominal_length_flag = false; }
    block->recalculate_flag = true; // Always calculate trapezoid for new block
  
    // Update previous path unit_vector and nominal speed
    memcpy(previous_unit_vec, unit_vec, sizeof(unit_vec)); // previous_unit_vec[] = unit_vec[]
    previous_nominal_speed = block->nominal_speed;

  } else {
    // Acceleration planner disabled. Set minimum that is required.
    block->initial_rate = block->nominal_rate;
    block->final_rate = block->nominal_rate;
    block->accelerate_until = 0;
    block->decelerate_after = block->step_event_count;
    block->rate_delta = 0;
  }
  
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
