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

static block_t block_buffer[BLOCK_BUFFER_SIZE];  // A ring buffer for motion instructions
static volatile uint8_t block_buffer_head;       // Index of the next block to be pushed
static volatile uint8_t block_buffer_tail;       // Index of the block to process now
static uint8_t next_buffer_head;                 // Index of the next buffer head

// Define planner variables
typedef struct {
  int32_t position[N_AXIS];        // The planner position of the tool in absolute steps. Kept separate
                                   // from g-code position for movements requiring multiple line motions,
                                   // i.e. arcs, canned cycles, and backlash compensation.
  float previous_unit_vec[N_AXIS]; // Unit vector of previous path line segment
  float previous_nominal_speed;    // Nominal speed of previous path line segment
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


// Calculates the final velocity from an initial velocity over a distance with constant acceleration.
// NOTE: Guaranteed to not exceed BLOCK_BUFFER_SIZE calls per planner cycle.
static float calculate_final_velocity(float acceleration, float initial_velocity, float distance) 
{
  return( sqrt(initial_velocity*initial_velocity+2*acceleration*distance) );
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
static void calculate_trapezoid_for_block(block_t *block, float entry_speed, float exit_speed) 
{  
  // Compute new initial rate for stepper algorithm
  block->initial_rate = ceil(entry_speed*(RANADE_MULTIPLIER/(60*ISR_TICKS_PER_SECOND))); // (mult*mm/isr_tic)
    
  // Compute efficiency variable for following calculations. Removes a float divide and multiply.
  // TODO: If memory allows, this can be kept in the block buffer since it doesn't change, even after feed holds.
  float steps_per_mm_div_2_acc = block->step_event_count/(2*settings.acceleration*block->millimeters); 
  
  // First determine intersection distance (in steps) from the exit point for a triangular profile.
  // Computes: steps_intersect = steps/mm * ( distance/2 + (v_entry^2-v_exit^2)/(4*acceleration) )
  int32_t intersect_distance = ceil( 0.5*(block->step_event_count + steps_per_mm_div_2_acc*(entry_speed*entry_speed-exit_speed*exit_speed)) );  
  
  // Check if this is a pure acceleration block by a intersection distance less than zero. Also
  // prevents signed and unsigned integer conversion errors.
  if (intersect_distance <= 0) {
    block->decelerate_after = 0; 
  } else {
    // Determine deceleration distance (in steps) from nominal speed to exit speed for a trapezoidal profile.
    // Value is never negative. Nominal speed is always greater than or equal to the exit speed.
    // Computes: steps_decelerate = steps/mm * ( (v_nominal^2 - v_exit^2)/(2*acceleration) )
    block->decelerate_after = ceil(steps_per_mm_div_2_acc * 
                                   (block->nominal_speed*block->nominal_speed-exit_speed*exit_speed));

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

  NOTE: As executing blocks complete and incoming streaming blocks are appended to the planner buffer, this
  function is constantly re-calculating and must be as efficient as possible. For example, in situations like 
  arc generation or complex curves, the short, rapid line segments can execute faster than new blocks can be 
  added, and the planner buffer will starve and empty, leading to weird hiccup-like jerky motions.
*/
static void planner_recalculate() 
{     
  // TODO: No over-write protection exists for the executing block. For most cases this has proven to be ok, but 
  // for feed-rate overrides, something like this is essential. Place a request here to the stepper driver to
  // find out where in the planner buffer is the a safe place to begin re-planning from.

  // Perform reverse planner pass. Skip the head(end) block since it is already initialized, and skip the
  // tail(first) block to prevent over-writing of the initial entry speed.
  uint8_t block_index = block_buffer_head;
  block_t *current = &block_buffer[block_index]; // Head block.
  block_t *next;
  if (block_index != block_buffer_tail) { block_index = prev_block_index( block_index ); }
  while (block_index != block_buffer_tail) {
    next = current;
    current = &block_buffer[block_index];    
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and 
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (current->entry_speed != current->max_entry_speed) {
      // If nominal length true, max junction speed is guaranteed to be reached. Only compute
      // for max allowable speed if block is decelerating and nominal length is false.
      if ((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed)) {
        current->entry_speed = min( current->max_entry_speed,
          calculate_final_velocity(settings.acceleration,next->entry_speed,current->millimeters)); // Back-compute
      } else {
        current->entry_speed = current->max_entry_speed;
      } 
      current->recalculate_flag = true;        
    }
    block_index = prev_block_index( block_index );
  } 
  
  // Perform forward planner pass. Begins junction speed adjustments after tail(first) block.
  // Also recalculate trapezoids, block by block, as the forward pass completes the plan.
  block_index = block_buffer_tail;
  next = NULL;
  while (block_index != block_buffer_head) {
    current = next;
    next= &block_buffer[block_index];

    if (current) {
    
      // If the current block is an acceleration block, but it is not long enough to complete the
      // full speed change within the block, we need to adjust the entry speed accordingly. Entry
      // speeds have already been reset, maximized, and reverse planned by reverse planner.
      // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.  
      if (!current->nominal_length_flag) {
        if (current->entry_speed < next->entry_speed) {
          float entry_speed = min( next->entry_speed,
            calculate_final_velocity(settings.acceleration,current->entry_speed,current->millimeters) );
    
          // Check for junction speed change
          if (next->entry_speed != entry_speed) {
            next->entry_speed = entry_speed;
            next->recalculate_flag = true;
          }
        }
        
        // Recalculate if current block entry or exit junction speed has changed.
        if (current->recalculate_flag || next->recalculate_flag) {
          // NOTE: Entry and exit factors always > 0 by all previous logic operations.     
          calculate_trapezoid_for_block(current, current->entry_speed, next->entry_speed);      
          current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
        }
      }      
      
    }
    block_index = next_block_index( block_index );
  }
  // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
  calculate_trapezoid_for_block(next, next->entry_speed, MINIMUM_PLANNER_SPEED);
  next->recalculate_flag = false;
}

void plan_reset_buffer() 
{
  block_buffer_tail = block_buffer_head;
  next_buffer_head = next_block_index(block_buffer_head);
}

void plan_init() 
{
  plan_reset_buffer();
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
  if (block_buffer_tail == next_buffer_head) { return(true); }
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
void plan_buffer_line(float x, float y, float z, float feed_rate, uint8_t invert_feed_rate) 
{
  // Prepare to set up new block
  block_t *block = &block_buffer[block_buffer_head];

  // Calculate target position in absolute steps
  int32_t target[3];
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
  // TODO: Store last xyz in memory to remove steps_per_mm divides. Or come up with another way to save cycles.
  float delta_mm[3];
  delta_mm[X_AXIS] = block->steps_x/settings.steps_per_mm[X_AXIS];
  delta_mm[Y_AXIS] = block->steps_y/settings.steps_per_mm[Y_AXIS];
  delta_mm[Z_AXIS] = block->steps_z/settings.steps_per_mm[Z_AXIS];
  block->millimeters = sqrt(delta_mm[X_AXIS]*delta_mm[X_AXIS] + delta_mm[Y_AXIS]*delta_mm[Y_AXIS] + 
                            delta_mm[Z_AXIS]*delta_mm[Z_AXIS]);
  float inverse_millimeters = 1.0/block->millimeters;  // Inverse millimeters to remove multiple divides	

  // Compute path unit vector                            
  float unit_vec[3];
  unit_vec[X_AXIS] = delta_mm[X_AXIS]*inverse_millimeters;
  unit_vec[Y_AXIS] = delta_mm[Y_AXIS]*inverse_millimeters;
  unit_vec[Z_AXIS] = delta_mm[Z_AXIS]*inverse_millimeters;  

  // Compute direction bits and correct unit vector directions
  block->direction_bits = 0;
  if (target[X_AXIS] < pl.position[X_AXIS]) { 
    block->direction_bits |= (1<<X_DIRECTION_BIT); 
    unit_vec[X_AXIS] = -unit_vec[X_AXIS];
  }
  if (target[Y_AXIS] < pl.position[Y_AXIS]) {
    block->direction_bits |= (1<<Y_DIRECTION_BIT);
    unit_vec[Y_AXIS] = -unit_vec[Y_AXIS];
  }
  if (target[Z_AXIS] < pl.position[Z_AXIS]) { 
    block->direction_bits |= (1<<Z_DIRECTION_BIT);
    unit_vec[Z_AXIS] = -unit_vec[Z_AXIS];
  }
  
  // Calculate speed in mm/minute for each axis. No divide by zero due to previous checks.
  // NOTE: Minimum stepper speed is limited by MINIMUM_STEPS_PER_MINUTE in stepper.c
  if (invert_feed_rate) { feed_rate = block->millimeters/feed_rate; }
  block->nominal_speed = feed_rate; // (mm/min) Always > 0
  
  // TODO: When acceleration independence is installed, it can be kept in terms of 2*acceleration for 
  // each block. This could save some 2*acceleration multiplications elsewhere. Need to check though.
  
  // Compute the acceleration, nominal rate, and distance traveled per step event for the stepper algorithm.
  block->rate_delta = ceil(settings.acceleration*
    ((RANADE_MULTIPLIER/(60*60))/(ISR_TICKS_PER_SECOND*ACCELERATION_TICKS_PER_SECOND))); // (mult*mm/isr_tic/accel_tic)
  block->nominal_rate = ceil(block->nominal_speed*(RANADE_MULTIPLIER/(60.0*ISR_TICKS_PER_SECOND))); // (mult*mm/isr_tic)
  block->d_next = ceil((block->millimeters*RANADE_MULTIPLIER)/block->step_event_count); // (mult*mm/step)
  
  // Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
  // Let a circle be tangent to both previous and current path line segments, where the junction 
  // deviation is defined as the distance from the junction to the closest edge of the circle, 
  // colinear with the circle center. The circular segment joining the two paths represents the 
  // path of centripetal acceleration. Solve for max velocity based on max acceleration about the
  // radius of the circle, defined indirectly by junction deviation. This may be also viewed as 
  // path width or max_jerk in the previous grbl version. This approach does not actually deviate 
  // from path, but used as a robust way to compute cornering speeds, as it takes into account the
  // nonlinearities of both the junction angle and junction velocity.
  // NOTE: This is basically an exact path mode (G61), but it doesn't come to a complete stop unless
  // the junction deviation value is high. In the future, if continuous mode (G64) is desired, the
  // math here is exactly the same. Instead of motioning all the way to junction point, the machine
  // will just need to follow the arc circle defined above and check if the arc radii are no longer
  // than half of either line segment to ensure no overlapping. Right now, the Arduino likely doesn't
  // have the horsepower to do these calculations at high feed rates.
  float vmax_junction = MINIMUM_PLANNER_SPEED; // Set default max junction speed

  // Skip first block or when previous_nominal_speed is used as a flag for homing and offset cycles.
  if ((block_buffer_head != block_buffer_tail) && (pl.previous_nominal_speed > 0.0)) {
    // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
    // NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
    float cos_theta = - pl.previous_unit_vec[X_AXIS] * unit_vec[X_AXIS] 
                      - pl.previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS] 
                      - pl.previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS] ;
                        
    // Skip and use default max junction speed for 0 degree acute junction.
    if (cos_theta < 0.95) {
      vmax_junction = min(pl.previous_nominal_speed,block->nominal_speed);
      // Skip and avoid divide by zero for straight junctions at 180 degrees. Limit to min() of nominal speeds.
      if (cos_theta > -0.95) {
        // Compute maximum junction velocity based on maximum acceleration and junction deviation
        float sin_theta_d2 = sqrt(0.5*(1.0-cos_theta)); // Trig half angle identity. Always positive.
        vmax_junction = min(vmax_junction,
          sqrt(settings.acceleration * settings.junction_deviation * sin_theta_d2/(1.0-sin_theta_d2)) );
      }
    }
  }
  block->max_entry_speed = vmax_junction;
  
  // Initialize block entry speed. Compute block entry velocity backwards from user-defined MINIMUM_PLANNER_SPEED.
  float v_allowable = calculate_final_velocity(settings.acceleration,MINIMUM_PLANNER_SPEED,block->millimeters);
  block->entry_speed = min(vmax_junction, v_allowable);

  // Initialize planner efficiency flags
  // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
  // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
  // the current block and next block junction speeds are guaranteed to always be at their maximum
  // junction speeds in deceleration and acceleration, respectively. This is due to how the current
  // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
  // the reverse and forward planners, the corresponding block junction speed will always be at the
  // the maximum junction speed and may always be ignored for any speed reduction checks.
  if (block->nominal_speed <= v_allowable) { block->nominal_length_flag = true; }
  else { block->nominal_length_flag = false; }
  block->recalculate_flag = true; // Always calculate trapezoid for new block

  // Update previous path unit_vector and nominal speed
  memcpy(pl.previous_unit_vec, unit_vec, sizeof(unit_vec)); // pl.previous_unit_vec[] = unit_vec[]
  pl.previous_nominal_speed = block->nominal_speed;
  
  // Update buffer head and next buffer head indices
  block_buffer_head = next_buffer_head;  
  next_buffer_head = next_block_index(block_buffer_head);
  
  // Update planner position
  memcpy(pl.position, target, sizeof(target)); // pl.position[] = target[]

  planner_recalculate(); 
}

// Reset the planner position vector (in steps). Called by the system abort routine.
void plan_set_current_position(int32_t x, int32_t y, int32_t z)
{
  pl.position[X_AXIS] = x;
  pl.position[Y_AXIS] = y;
  pl.position[Z_AXIS] = z;
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
  block->entry_speed = 0.0;
  block->max_entry_speed = 0.0;
  block->nominal_length_flag = false;
  block->recalculate_flag = true;
  planner_recalculate();  
}
