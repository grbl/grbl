/*
  planner.c - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011-2012 Sungeun K. Jeon
  Copyright (c) 2011 Jens Geisler  
  
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
#include <stdlib.h>
#include "planner.h"
#include "nuts_bolts.h"
#include "stepper.h"
#include "settings.h"
#include "config.h"
#include "protocol.h"

#define SOME_LARGE_VALUE 1.0E+38 // Used by rapids and acceleration maximization calculations. Just needs
                                 // to be larger than any feasible (mm/min)^2 or mm/sec^2 value.

static block_t block_buffer[BLOCK_BUFFER_SIZE];  // A ring buffer for motion instructions
static volatile uint8_t block_buffer_head;       // Index of the next block to be pushed
static volatile uint8_t block_buffer_tail;       // Index of the block to process now
static uint8_t next_buffer_head;                 // Index of the next buffer head
// static *block_t block_buffer_planned;

// Define planner variables
typedef struct {
  int32_t position[N_AXIS];          // The planner position of the tool in absolute steps. Kept separate
                                     // from g-code position for movements requiring multiple line motions,
                                     // i.e. arcs, canned cycles, and backlash compensation.
  float previous_unit_vec[N_AXIS];   // Unit vector of previous path line segment
  float previous_nominal_speed_sqr;  // Nominal speed of previous path line segment
  float last_x, last_y, last_z;      // Target position of previous path line segment
} planner_t;
static planner_t pl;


// Returns the index of the next block in the ring buffer
// NOTE: Removed modulo (%) operator, which uses an expensive divide and multiplication.
static uint8_t next_block_index(uint8_t block_index) 
{
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) { block_index = 0; }
  return(block_index);
}


// Returns the index of the previous block in the ring buffer
static uint8_t prev_block_index(uint8_t block_index) 
{
  if (block_index == 0) { block_index = BLOCK_BUFFER_SIZE; }
  block_index--;
  return(block_index);
}


/*       STEPPER VELOCITY PROFILE DEFINITION 
                                                  less than nominal rate->  + 
                    +--------+ <- nominal_rate                             /|\                                         
                   /          \                                           / | \                      
  initial_rate -> +            \                                         /  |  + <- next->initial_rate         
                  |             + <- next->initial_rate                 /   |  |                       
                  +-------------+                      initial_rate -> +----+--+                   
                   time -->  ^  ^                                           ^  ^                       
                             |  |                                           |  |                       
                     decelerate distance                           decelerate distance  
                                                        
  Calculates trapezoid parameters for stepper algorithm. Each block velocity profiles can be 
  described as either a trapezoidal or a triangular shape. The trapezoid occurs when the block
  reaches the nominal speed of the block and cruises for a period of time. A triangle occurs
  when the nominal speed is not reached within the block. Some other special cases exist, 
  such as pure ac/de-celeration velocity profiles from beginning to end or a trapezoid that
  has no deceleration period when the next block resumes acceleration. 
  
  The following function determines the type of velocity profile and stores the minimum required
  information for the stepper algorithm to execute the calculated profiles. In this case, only
  the new initial rate and n_steps until deceleration are computed, since the stepper algorithm
  already handles acceleration and cruising and just needs to know when to start decelerating.
*/
static void calculate_trapezoid_for_block(block_t *block, float entry_speed_sqr, float exit_speed_sqr) 
{  
  // Compute new initial rate for stepper algorithm
  block->initial_rate = ceil(sqrt(entry_speed_sqr)*(RANADE_MULTIPLIER/(60*ISR_TICKS_PER_SECOND))); // (mult*mm/isr_tic)
          
  // TODO: Compute new nominal rate if a feedrate override occurs.
  // block->nominal_rate = ceil(feed_rate*(RANADE_MULTIPLIER/(60.0*ISR_TICKS_PER_SECOND))); // (mult*mm/isr_tic)
    
  // Compute efficiency variable for following calculations. Removes a float divide and multiply.
  // TODO: If memory allows, this can be kept in the block buffer since it doesn't change, even after feed holds.
  float steps_per_mm_div_2_acc = block->step_event_count/(2*block->acceleration*block->millimeters); 
  
  // First determine intersection distance (in steps) from the exit point for a triangular profile.
  // Computes: steps_intersect = steps/mm * ( distance/2 + (v_entry^2-v_exit^2)/(4*acceleration) )
  int32_t intersect_distance = ceil( 0.5*(block->step_event_count + steps_per_mm_div_2_acc*(entry_speed_sqr-exit_speed_sqr)) );  
  
  // Check if this is a pure acceleration block by a intersection distance less than zero. Also
  // prevents signed and unsigned integer conversion errors.
  if (intersect_distance <= 0) {
    block->decelerate_after = 0; 
  } else {
    // Determine deceleration distance (in steps) from nominal speed to exit speed for a trapezoidal profile.
    // Value is never negative. Nominal speed is always greater than or equal to the exit speed.
    // Computes: steps_decelerate = steps/mm * ( (v_nominal^2 - v_exit^2)/(2*acceleration) )
    block->decelerate_after = ceil(steps_per_mm_div_2_acc * (block->nominal_speed_sqr - exit_speed_sqr));

    // The lesser of the two triangle and trapezoid distances always defines the velocity profile.
    if (block->decelerate_after > intersect_distance) { block->decelerate_after = intersect_distance; }
    
    // Finally, check if this is a pure deceleration block.
    if (block->decelerate_after > block->step_event_count) { block->decelerate_after = block->step_event_count; }
  }
}     
                                        

/*                            PLANNER SPEED DEFINITION                                              
                                     +--------+   <- current->nominal_speed
                                    /          \                                
         current->entry_speed ->   +            \                               
                                   |             + <- next->entry_speed
                                   +-------------+                              
                                       time -->                      
                                                  
  Recalculates the motion plan according to the following algorithm:
  
    1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_speed) 
       so that:
      a. The junction speed is equal to or less than the maximum junction speed limit
      b. No speed reduction within one block requires faster deceleration than the acceleration limits.
      c. The last (or newest appended) block is planned from a complete stop.
    2. Go over every block in chronological (forward) order and dial down junction speed values if 
      a. The speed increase within one block would require faster acceleration than the acceleration limits.
  
  When these stages are complete, all blocks have a junction entry speed that will allow all speed changes
  to be performed using the overall limiting acceleration value, and where no junction speed is greater
  than the max limit. In other words, it just computed the fastest possible velocity profile through all 
  buffered blocks, where the final buffered block is planned to come to a full stop when the buffer is fully
  executed. Finally it will:
  
    3. Convert the plan to data that the stepper algorithm needs. Only block trapezoids adjacent to a
       a planner-modified junction speed with be updated, the others are assumed ok as is.
  
  All planner computations(1)(2) are performed in floating point to minimize numerical round-off errors. Only
  when planned values are converted to stepper rate parameters(3), these are integers. If another motion block
  is added while executing, the planner will re-plan and update the stored optimal velocity profile as it goes.
  
  Conceptually, the planner works like blowing up a balloon, where the balloon is the velocity profile. It's
  constrained by the speeds at the beginning and end of the buffer, along with the maximum junction speeds and
  nominal speeds of each block. Once a plan is computed, or balloon filled, this is the optimal velocity profile
  through all of the motions in the buffer. Whenever a new block is added, this changes some of the limiting
  conditions, or how the balloon is filled, so it has to be re-calculated to get the new optimal velocity profile.
  
  Also, since the planner only computes on what's in the planner buffer, some motions with lots of short line
  segments, like arcs, may seem to move slow. This is because there simply isn't enough combined distance traveled 
  in the entire buffer to accelerate up to the nominal speed and then decelerate to a stop at the end of the
  buffer. There are a few simple solutions to this: (1) Maximize the machine acceleration. The planner will be 
  able to compute higher speed profiles within the same combined distance. (2) Increase line segment(s) distance.
  The more combined distance the planner has to use, the faster it can go. (3) Increase the MINIMUM_PLANNER_SPEED.
  Not recommended. This will change what speed the planner plans to at the end of the buffer. Can lead to lost 
  steps when coming to a stop. (4) [BEST] Increase the planner buffer size. The more combined distance, the 
  bigger the balloon, or faster it can go. But this is not possible for 328p Arduinos because its limited memory 
  is already maxed out. Future ARM versions should not have this issue, with look-ahead planner blocks numbering 
  up to a hundred or more.

  NOTE: Since this function is constantly re-calculating for every new incoming block, it must be as efficient
  as possible. For example, in situations like arc generation or complex curves, the short, rapid line segments
  can execute faster than new blocks can be added, and the planner buffer will then starve and empty, leading
  to weird hiccup-like jerky motions.
*/
static void planner_recalculate() 
{     

//   float entry_speed_sqr;
//   uint8_t block_index = block_buffer_head;
//   block_t *previous = NULL;
//   block_t *current = NULL;
//   block_t *next;
//   while (block_index != block_buffer_tail) {
//     block_index = prev_block_index( block_index );
//     next = current;
//     current = previous;
//     previous = &block_buffer[block_index];
//     
//     if (next && current) {
//       if (next != block_buffer_planned) {
//         if (previous == block_buffer_tail) { block_buffer_planned = next; }
//         else {
//         
//           if (current->entry_speed_sqr != current->max_entry_speed_sqr) {
//             current->recalculate_flag = true; // Almost always changes. So force recalculate.       
//             entry_speed_sqr = next->entry_speed_sqr + 2*current->acceleration*current->millimeters;
//             if (entry_speed_sqr < current->max_entry_speed_sqr) {
//               current->entry_speed_sqr = entry_speed_sqr;
//             } else {
//               current->entry_speed_sqr = current->max_entry_speed_sqr;
//             }
//           } else {  
//             block_buffer_planned = current;
//           }
//         }
//       } else { 
//         break;
//       }
//     }
//   } 
// 
//   block_index = block_buffer_planned;
//   next = &block_buffer[block_index];
//   current = prev_block_index(block_index);
//   while (block_index != block_buffer_head) {
// 
//       // If the current block is an acceleration block, but it is not long enough to complete the
//       // full speed change within the block, we need to adjust the exit speed accordingly. Entry
//       // speeds have already been reset, maximized, and reverse planned by reverse planner.
//       if (current->entry_speed_sqr < next->entry_speed_sqr) {
//         // Compute block exit speed based on the current block speed and distance
//         // Computes: v_exit^2 = v_entry^2 + 2*acceleration*distance
//         entry_speed_sqr = current->entry_speed_sqr + 2*current->acceleration*current->millimeters;
//         
//         // If it's less than the stored value, update the exit speed and set recalculate flag.
//         if (entry_speed_sqr < next->entry_speed_sqr) {
//           next->entry_speed_sqr = entry_speed_sqr;
//           next->recalculate_flag = true;
//         }
//       }
// 
//       // Recalculate if current block entry or exit junction speed has changed.
//       if (current->recalculate_flag || next->recalculate_flag) {
//         // NOTE: Entry and exit factors always > 0 by all previous logic operations.     
//         calculate_trapezoid_for_block(current, current->entry_speed_sqr, next->entry_speed_sqr);      
//         current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
//       }
//       
//     current = next;
//     next = &block_buffer[block_index];
//     block_index = next_block_index( block_index );
//   }
//   
//   // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
//   calculate_trapezoid_for_block(next, next->entry_speed_sqr, MINIMUM_PLANNER_SPEED*MINIMUM_PLANNER_SPEED);
//   next->recalculate_flag = false;
  
  // TODO: No over-write protection exists for the executing block. For most cases this has proven to be ok, but 
  // for feed-rate overrides, something like this is essential. Place a request here to the stepper driver to
  // find out where in the planner buffer is the a safe place to begin re-planning from.

//   if (block_buffer_head != block_buffer_tail) {   
  float entry_speed_sqr;

  // Perform reverse planner pass. Skip the head(end) block since it is already initialized, and skip the
  // tail(first) block to prevent over-writing of the initial entry speed. 
  uint8_t block_index = prev_block_index( block_buffer_head ); // Assume buffer is not empty.
  block_t *current = &block_buffer[block_index]; // Head block-1 = Newly appended block
  block_t *next;
  if (block_index != block_buffer_tail) { block_index = prev_block_index( block_index ); }
  while (block_index != block_buffer_tail) {
    next = current;
    current = &block_buffer[block_index];
    
    // TODO: Determine maximum entry speed at junction for feedrate overrides, since they can alter
    // the planner nominal speeds at any time. This calc could be done in the override handler, but 
    // this could require an additional variable to be stored to differentiate the programmed nominal
    // speeds, max junction speed, and override speeds/scalar.
    
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and 
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (current->entry_speed_sqr != current->max_entry_speed_sqr) {

      current->entry_speed_sqr = current->max_entry_speed_sqr;
      current->recalculate_flag = true; // Almost always changes. So force recalculate.       

      if (next->entry_speed_sqr < current->max_entry_speed_sqr) {
        // Computes: v_entry^2 = v_exit^2 + 2*acceleration*distance      
        entry_speed_sqr = next->entry_speed_sqr + 2*current->acceleration*current->millimeters;
        if (entry_speed_sqr < current->max_entry_speed_sqr) {
          current->entry_speed_sqr = entry_speed_sqr;
        }
      } 
    }    
    block_index = prev_block_index( block_index );
  } 

  // Perform forward planner pass. Begins junction speed adjustments after tail(first) block.
  // Also recalculate trapezoids, block by block, as the forward pass completes the plan.
  block_index = next_block_index(block_buffer_tail);
  next = &block_buffer[block_buffer_tail]; // Places tail(first) block into current
  while (block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];

      // If the current block is an acceleration block, but it is not long enough to complete the
      // full speed change within the block, we need to adjust the exit speed accordingly. Entry
      // speeds have already been reset, maximized, and reverse planned by reverse planner.
      if (current->entry_speed_sqr < next->entry_speed_sqr) {
        // Compute block exit speed based on the current block speed and distance
        // Computes: v_exit^2 = v_entry^2 + 2*acceleration*distance
        entry_speed_sqr = current->entry_speed_sqr + 2*current->acceleration*current->millimeters;
        
        // If it's less than the stored value, update the exit speed and set recalculate flag.
        if (entry_speed_sqr < next->entry_speed_sqr) {
          next->entry_speed_sqr = entry_speed_sqr;
          next->recalculate_flag = true;
        }
      }

      // Recalculate if current block entry or exit junction speed has changed.
      if (current->recalculate_flag || next->recalculate_flag) {
        // NOTE: Entry and exit factors always > 0 by all previous logic operations.     
        calculate_trapezoid_for_block(current, current->entry_speed_sqr, next->entry_speed_sqr);      
        current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
      }

    block_index = next_block_index( block_index );
  }
  
  // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
  calculate_trapezoid_for_block(next, next->entry_speed_sqr, MINIMUM_PLANNER_SPEED*MINIMUM_PLANNER_SPEED);
  next->recalculate_flag = false;
//   }
}

void plan_init() 
{
  block_buffer_tail = block_buffer_head;
  next_buffer_head = next_block_index(block_buffer_head);
//   block_buffer_planned = block_buffer_head;
  memset(&pl, 0, sizeof(pl)); // Clear planner struct
}

inline void plan_discard_current_block() 
{
  if (block_buffer_head != block_buffer_tail) {
    block_buffer_tail = next_block_index( block_buffer_tail );
  }
}

inline block_t *plan_get_current_block() 
{
  if (block_buffer_head == block_buffer_tail) { return(NULL); }
  return(&block_buffer[block_buffer_tail]);
}

// Returns the availability status of the block ring buffer. True, if full.
uint8_t plan_check_full_buffer()
{
  if (block_buffer_tail == next_buffer_head) { 
    // TODO: Move this back into motion control. Shouldn't be here, but it's efficient.
    if (sys.auto_start) { st_cycle_start(); } // Auto-cycle start when buffer is full.
    return(true); 
  }
  return(false);
}

// Block until all buffered steps are executed or in a cycle state. Works with feed hold
// during a synchronize call, if it should happen. Also, waits for clean cycle end.
void plan_synchronize()
{
  while (plan_get_current_block() || sys.state == STATE_CYCLE) { 
    protocol_execute_runtime();   // Check and execute run-time commands
    if (sys.abort) { return; } // Check for system abort
  }    
}

// Add a new linear movement to the buffer. x, y and z is the signed, absolute target position in 
// millimeters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
// rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
// All position data passed to the planner must be in terms of machine position to keep the planner 
// independent of any coordinate system changes and offsets, which are handled by the g-code parser.
// NOTE: Assumes buffer is available. Buffer checks are handled at a higher level by motion_control.
// Also the feed rate input value is used in three ways: as a normal feed rate if invert_feed_rate
// is false, as inverse time if invert_feed_rate is true, or as seek/rapids rate if the feed_rate
// value is negative (and invert_feed_rate always false).
void plan_buffer_line(float x, float y, float z, float feed_rate, uint8_t invert_feed_rate) 
{
  // Prepare to set up new block
  block_t *block = &block_buffer[block_buffer_head];

  // Calculate target position in absolute steps
  int32_t target[N_AXIS];
  target[X_AXIS] = lround(x*settings.steps_per_mm[X_AXIS]);
  target[Y_AXIS] = lround(y*settings.steps_per_mm[Y_AXIS]);
  target[Z_AXIS] = lround(z*settings.steps_per_mm[Z_AXIS]);     
  
  // Number of steps for each axis
  block->steps_x = labs(target[X_AXIS]-pl.position[X_AXIS]);
  block->steps_y = labs(target[Y_AXIS]-pl.position[Y_AXIS]);
  block->steps_z = labs(target[Z_AXIS]-pl.position[Z_AXIS]);
  block->step_event_count = max(block->steps_x, max(block->steps_y, block->steps_z));

  // Bail if this is a zero-length block
  if (block->step_event_count == 0) { return; };
  
  // Compute path vector in terms of absolute step target and current positions
  float delta_mm[N_AXIS];
  delta_mm[X_AXIS] = x-pl.last_x;
  delta_mm[Y_AXIS] = y-pl.last_y; 
  delta_mm[Z_AXIS] = z-pl.last_z;
  block->millimeters = sqrt(delta_mm[X_AXIS]*delta_mm[X_AXIS] + delta_mm[Y_AXIS]*delta_mm[Y_AXIS] + 
                            delta_mm[Z_AXIS]*delta_mm[Z_AXIS]);

  // Adjust feed_rate value to mm/min depending on type of rate input (normal, inverse time, or rapids)
  // TODO: Need to distinguish a rapids vs feed move for overrides. Some flag of some sort.
  if (feed_rate < 0) { feed_rate = SOME_LARGE_VALUE; } // Scaled down to absolute max/rapids rate later
  else if (invert_feed_rate) { feed_rate = block->millimeters/feed_rate; }

  // Calculate the unit vector of the line move and the block maximum feed rate and acceleration limited
  // by the maximum possible values. Block rapids rates are computed or feed rates are scaled down so
  // they don't exceed the maximum axes velocities. The block acceleration is maximized based on direction
  // and axes properties as well.
  // NOTE: This calculation assumes all axes are orthogonal (Cartesian) and works with ABC-axes,
  // if they are also orthogonal/independent. Operates on the absolute value of the unit vector.
  uint8_t i;
  float unit_vec[N_AXIS], inverse_unit_vec_value;
  float inverse_millimeters = 1.0/block->millimeters;  // Inverse millimeters to remove multiple float divides	
  block->acceleration = SOME_LARGE_VALUE; // Scaled down to maximum acceleration in loop
  for (i=0; i<N_AXIS; i++) { 
    if (delta_mm[i] == 0) { 
      unit_vec[i] = 0;  // Store zero value. And avoid divide by zero.
    } else {
      // Compute unit vector and its absolute inverse value
      unit_vec[i] = delta_mm[i]*inverse_millimeters;
      inverse_unit_vec_value = abs(1.0/unit_vec[i]);
      // Check and limit feed rate against max axis velocities and scale accelerations to maximums
      feed_rate = min(feed_rate,settings.max_velocity[i]*inverse_unit_vec_value);
      block->acceleration = min(block->acceleration,settings.acceleration[i]*inverse_unit_vec_value);
    }
  }

  // Compute nominal speed and rates
  block->nominal_speed_sqr = feed_rate*feed_rate; // (mm/min)^2. Always > 0
  block->nominal_rate = ceil(feed_rate*(RANADE_MULTIPLIER/(60.0*ISR_TICKS_PER_SECOND))); // (mult*mm/isr_tic)

  // Compute the acceleration and distance traveled per step event for the stepper algorithm.
  block->rate_delta = ceil(block->acceleration*
    ((RANADE_MULTIPLIER/(60.0*60.0))/(ISR_TICKS_PER_SECOND*ACCELERATION_TICKS_PER_SECOND))); // (mult*mm/isr_tic/accel_tic)
  block->d_next = ceil((block->millimeters*RANADE_MULTIPLIER)/block->step_event_count); // (mult*mm/step)
  
  // Compute direction bits. Bit enabled always means direction is negative.
  block->direction_bits = 0;
  if (unit_vec[X_AXIS] < 0) { block->direction_bits |= (1<<X_DIRECTION_BIT); }
  if (unit_vec[Y_AXIS] < 0) { block->direction_bits |= (1<<Y_DIRECTION_BIT); }
  if (unit_vec[Z_AXIS] < 0) { block->direction_bits |= (1<<Z_DIRECTION_BIT); }

  // Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
  // Let a circle be tangent to both previous and current path line segments, where the junction 
  // deviation is defined as the distance from the junction to the closest edge of the circle, 
  // colinear with the circle center. The circular segment joining the two paths represents the 
  // path of centripetal acceleration. Solve for max velocity based on max acceleration about the
  // radius of the circle, defined indirectly by junction deviation. This may be also viewed as 
  // path width or max_jerk in the previous grbl version. This approach does not actually deviate 
  // from path, but used as a robust way to compute cornering speeds, as it takes into account the
  // nonlinearities of both the junction angle and junction velocity.
  // NOTE: If the junction deviation value is finite, Grbl executes the motions in an exact path 
  // mode (G61). If the junction deviation value is zero, Grbl will execute the motion in an exact
  // stop mode (G61.1) manner. In the future, if continuous mode (G64) is desired, the math here
  // is exactly the same. Instead of motioning all the way to junction point, the machine will
  // just follow the arc circle defined here. The Arduino doesn't have the CPU cycles to perform
  // a continuous mode path, but ARM-based microcontrollers most certainly do.

  // Skip first block or when previous_nominal_speed is used as a flag for homing and offset cycles.
  block->max_entry_speed_sqr = MINIMUM_PLANNER_SPEED*MINIMUM_PLANNER_SPEED;
  if ((block_buffer_head != block_buffer_tail) && (pl.previous_nominal_speed_sqr > 0.0)) {
    // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
    // NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
    float cos_theta = - pl.previous_unit_vec[X_AXIS] * unit_vec[X_AXIS] 
                      - pl.previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS] 
                      - pl.previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS] ;
                        
    // Skip and use default max junction speed for 0 degree acute junction.
    if (cos_theta < 0.95) {
      block->max_entry_speed_sqr = min(block->nominal_speed_sqr,pl.previous_nominal_speed_sqr);
      // Skip and avoid divide by zero for straight junctions at 180 degrees. Limit to min() of nominal speeds.
      if (cos_theta > -0.95) {
        // Compute maximum junction velocity based on maximum acceleration and junction deviation
        float sin_theta_d2 = sqrt(0.5*(1.0-cos_theta)); // Trig half angle identity. Always positive.
        block->max_entry_speed_sqr = min(block->max_entry_speed_sqr,
           block->acceleration * settings.junction_deviation * sin_theta_d2/(1.0-sin_theta_d2));
      }
    }
  }  

  // Initialize block entry speed. Compute block entry velocity backwards from user-defined MINIMUM_PLANNER_SPEED.
  // TODO: This could be moved to the planner recalculate function.
  block->entry_speed_sqr = min( block->max_entry_speed_sqr,
      MINIMUM_PLANNER_SPEED*MINIMUM_PLANNER_SPEED + 2*block->acceleration*block->millimeters);

  // Set new block to be recalculated for conversion to stepper data.
  block->recalculate_flag = true;

  // Update previous path unit_vector and nominal speed (squared)
  memcpy(pl.previous_unit_vec, unit_vec, sizeof(unit_vec)); // pl.previous_unit_vec[] = unit_vec[]
  pl.previous_nominal_speed_sqr = block->nominal_speed_sqr;
    
  // Update planner position
  memcpy(pl.position, target, sizeof(target)); // pl.position[] = target[]
  pl.last_x = x;
  pl.last_y = y;
  pl.last_z = z;

  // Update buffer head and next buffer head indices
  block_buffer_head = next_buffer_head;  
  next_buffer_head = next_block_index(block_buffer_head);

  planner_recalculate(); 
}

// Reset the planner position vectors. Called by the system abort/initialization routine.
void plan_set_current_position(int32_t x, int32_t y, int32_t z)
{
  pl.position[X_AXIS] = x;
  pl.position[Y_AXIS] = y;
  pl.position[Z_AXIS] = z;
  pl.last_x = x/settings.steps_per_mm[X_AXIS];
  pl.last_y = y/settings.steps_per_mm[Y_AXIS];
  pl.last_z = z/settings.steps_per_mm[Z_AXIS];
}

// Re-initialize buffer plan with a partially completed block, assumed to exist at the buffer tail.
// Called after a steppers have come to a complete stop for a feed hold and the cycle is stopped.
void plan_cycle_reinitialize(int32_t step_events_remaining) 
{
  block_t *block = &block_buffer[block_buffer_tail]; // Point to partially completed block
  
  // Only remaining millimeters and step_event_count need to be updated for planner recalculate. 
  // Other variables (step_x, step_y, step_z, rate_delta, etc.) all need to remain the same to
  // ensure the original planned motion is resumed exactly.
  block->millimeters = (block->millimeters*step_events_remaining)/block->step_event_count;
  block->step_event_count = step_events_remaining;
  
  // Re-plan from a complete stop. Reset planner entry speeds and flags.
  block->entry_speed_sqr = 0.0;
  block->max_entry_speed_sqr = 0.0;
  block->recalculate_flag = true;
  planner_recalculate();  
}
