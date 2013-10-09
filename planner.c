/*
  planner.c - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

  Copyright (c) 2011-2013 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
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

static plan_block_t block_buffer[BLOCK_BUFFER_SIZE];  // A ring buffer for motion instructions
static volatile uint8_t block_buffer_tail;       // Index of the block to process now
static uint8_t block_buffer_head;                // Index of the next block to be pushed
static uint8_t next_buffer_head;                 // Index of the next buffer head
static uint8_t block_buffer_planned;             // Index of the optimally planned block

// Define planner variables
typedef struct {
  int32_t position[N_AXIS];          // The planner position of the tool in absolute steps. Kept separate
                                     // from g-code position for movements requiring multiple line motions,
                                     // i.e. arcs, canned cycles, and backlash compensation.
  float previous_unit_vec[N_AXIS];   // Unit vector of previous path line segment
  float previous_nominal_speed_sqr;  // Nominal speed of previous path line segment
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
                            



// Update the entry speed and millimeters remaining to execute for a partially completed block. Called only
// when the planner knows it will be changing the conditions of this block.
// TODO: Set up to be called from planner calculations. Need supporting code framework still, i.e. checking
//   and executing this only when necessary, combine with the block_buffer_safe pointer.
// TODO: This is very similar to the planner reinitialize after a feed hold. Could make this do double duty.
void plan_update_partial_block(uint8_t block_index, float exit_speed_sqr)
{
// TODO: Need to make a condition to check if we need make these calculations. We don't if nothing has 
//   been executed or placed into segment buffer. This happens with the first block upon startup or if 
//   the segment buffer is exactly in between two blocks. Just check if the step_events_remaining is equal
//   the total step_event_count in the block. If so, we don't have to do anything.

  // !!! block index is the same as block_buffer_safe.
  // See if we can reduce this down to just requesting the millimeters remaining..
  uint8_t is_decelerating;
  float millimeters_remaining = 0.0;
  st_fetch_partial_block_parameters(block_index, &millimeters_remaining, &is_decelerating);
 
  if (millimeters_remaining != 0.0) {
    // Point to current block partially executed by stepper algorithm
    plan_block_t *partial_block = plan_get_block_by_index(block_index);

    // Compute the midway speed of the partially completely block at the end of the segment buffer.
    if (is_decelerating) { // Block is decelerating
      partial_block->entry_speed_sqr = exit_speed_sqr - 2*partial_block->acceleration*millimeters_remaining;
    } else { // Block is accelerating or cruising
      partial_block->entry_speed_sqr += 2*partial_block->acceleration*(partial_block->millimeters-millimeters_remaining);
      partial_block->entry_speed_sqr = min(partial_block->entry_speed_sqr, partial_block->nominal_speed_sqr);
    }

    // Update only the relevant planner block information so the planner can plan correctly.
    partial_block->millimeters = millimeters_remaining;
    partial_block->max_entry_speed_sqr = partial_block->entry_speed_sqr; // Not sure if this needs to be updated.
  }
}




/*                            PLANNER SPEED DEFINITION                                              
                                     +--------+   <- current->nominal_speed
                                    /          \                                
         current->entry_speed ->   +            \                               
                                   |             + <- next->entry_speed (aka exit speed)
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
  The more combined distance the planner has to use, the faster it can go. (3) Increase the MINIMUM_JUNCTION_SPEED.
  Not recommended. This will change what speed the planner plans to at the end of the buffer. Can lead to lost 
  steps when coming to a stop. (4) [BEST] Increase the planner buffer size. The more combined distance, the 
  bigger the balloon, or faster it can go. But this is not possible for 328p Arduinos because its limited memory 
  is already maxed out. Future ARM versions should not have this issue, with look-ahead planner blocks numbering 
  up to a hundred or more.

  NOTE: Since this function is constantly re-calculating for every new incoming block, it must be as efficient
  as possible. For example, in situations like arc generation or complex curves, the short, rapid line segments
  can execute faster than new blocks can be added, and the planner buffer will then starve and empty, leading
  to weird hiccup-like jerky motions.

  Index mapping:
  - block_buffer_head: Points to the newest incoming buffer block just added by plan_buffer_line(). The planner
      never touches the exit speed of this block, which always defaults to MINIMUM_JUNCTION_SPEED.
  - block_buffer_tail: Points to the beginning of the planner buffer. First to be executed or being executed. 
      Can dynamically change with the old stepper algorithm, but with the new algorithm, this should be impossible
      as long as the segment buffer is not empty.
  - next_buffer_head: Points to next planner buffer block after the last block. Should always be empty.
  - block_buffer_safe: Points to the first planner block in the buffer for which it is safe to change. Since
      the stepper can be executing the first block and if the planner changes its conditions, this will cause
      a discontinuity and error in the stepper profile with lost steps likely. With the new stepper algorithm,
      the block_buffer_safe is always where the stepper segment buffer ends and can never be overwritten, but
      this can change the state of the block profile from a pure trapezoid assumption. Meaning, if that block
      is decelerating, the planner conditions can change such that the block can new accelerate mid-block.
      
      !!! I need to make sure that the stepper algorithm can modify the acceleration mid-block. Needed for feedrate overrides too.
      
      !!! planner_recalculate() may not work correctly with re-planning.... may need to artificially set both the
          block_buffer_head and next_buffer_head back one index so that this works correctly, or allow the operation
          of this function to accept two different conditions to operate on.
          
  - block_buffer_planned: Points to the first buffer block after the last optimally fixed block, which can no longer be 
      improved. This block and the trailing buffer blocks that can still be altered when new blocks are added. This planned
      block points to the transition point between the fixed and non-fixed states and is handled slightly different. The entry
      speed is fixed, indicating the reverse pass cannot maximize the speed further, but the velocity profile within it
      can still be changed, meaning the forward pass calculations must start from here and influence the following block
      entry speed.

      !!! Need to check if this is the start of the non-optimal or the end of the optimal block.
*/
static void planner_recalculate() 
{   
  // Query stepper module for safe planner block index to recalculate to, which corresponds to the end
  // of the step segment buffer.
  uint8_t block_buffer_safe = st_get_prep_block_index();
  // TODO: Make sure that we don't have to check for the block_buffer_tail condition, if the stepper module
  // returns a NULL pointer or something. This could happen when the segment buffer is empty. Although,
  // this call won't return a NULL, only an index.. I have to make sure that this index is synced with the
  // planner at all times. 
  
  /* - In theory, the state of the segment buffer can exist anywhere within the planner buffer tail and head-1 
       or is empty, when there is nothing in the segment queue.  The safe pointer can be the buffer head only
       when the planner queue has been entirely queued into the segment buffer and there are no more blocks
       in the planner buffer. The segment buffer will to continue to execute the remainder of it, but the
       planner should be able to treat a newly added block during this time as an empty planner buffer since 
       we can't touch the segment buffer.
       
     - The segment buffer is atomic to the planner buffer, because the main program computes these seperately.
       Even if we move the planner head pointer early at the end of plan_buffer_line(), this shouldn't
       effect the safe pointer. 

     - If the safe pointer is at head-1, this means that the stepper algorithm has segments queued and may
       be executing. This is the last block in the planner queue, so it has been planned to decelerate to 
       zero at its end. When adding a new block, there will be at least two blocks to work with. When resuming,
       from a feed hold, we only have this block and will be computing nothing. The planner doesn't have to 
       do anything, since the trapezoid calculations called by the stepper module should complete the block plan.
  
     - In most cases, the safe pointer is at the plan tail or the block after, and rarely on the block two
       beyond the tail. Since the safe pointer points to the block used at the end of the segment buffer, it 
       can be in any one of these states. As the stepper module executes the planner block, the buffer tail, 
       and hence the safe pointer, can push forward through the planner blocks and overcome the planned
       pointer at any time. 

     - Does the reverse pass not touch either the safe or the plan pointer blocks? The plan pointer only 
       allows the velocity profile within it to be altered, but not the entry speed, so the reverse pass
       ignores this block. The safe pointer is the same way, where the entry speed does not change, but 
       the velocity profile within it does.
       
     - The planned pointer can exist anywhere in a given plan, except for the planner buffer head, if everything
       operates as anticipated. Since the planner buffer can be executed by the stepper algorithm as any
       rate and could empty the planner buffer quickly, the planner tail can overtake the planned pointer
       at any time, but will never go around the ring buffer and re-encounter itself, the plan itself is not
       changed by adding a new block or something else.
       
     - The planner recalculate function should always reset the planned pointer at the proper break points
       or when it encounters the safe block pointer, but will only do so when there are more than one block
       in the buffer. In the case of single blocks, the planned pointer should always be set to the first
       write-able block in the buffer, aka safe block. 

     - When does this not work? There might be an issue when the planned pointer moves from the tail to the
       next head as a new block is being added and planned. Otherwise, the planned pointer should remain 
       static within the ring buffer no matter what the buffer is doing: being executed, adding new blocks, 
       or both simultaneously. Need to make sure that this case is covered.
  */

  
  // Recompute plan only when there is more than one planner block in the buffer. Can't do anything with one.  
  // NOTE: block_buffer_safe can be equal to block_buffer_head if the segment buffer has completely queued up
  // the remainder of the planner buffer. In this case, a new planner block will be treated as a single block.
  if (block_buffer_head == block_buffer_safe) { // Also catches head = tail
    
    // Just set block_buffer_planned pointer.
    block_buffer_planned = block_buffer_head;
    printString("z");

    // TODO: Feedrate override of one block needs to update the partial block with an exit speed of zero. For
    //   a single added block and recalculate after a feed hold, we don't need to compute this, since we already
    //   know that the velocity starts and ends at zero. With an override, we can be traveling at some midblock
    //   rate, and we have to calculate the new velocity profile from it.
    // plan_update_partial_block(block_index,0.0);
           
  } else {
    
    // TODO: If the nominal speeds change during a feedrate override, we need to recompute the max entry speeds for 
    //       all junctions before proceeding.

    // Initialize planner buffer pointers and indexing.
    uint8_t block_index = block_buffer_head;
    plan_block_t *current = &block_buffer[block_index];

    // Calculate maximum entry speed for last block in buffer, where the exit speed is always zero.
    current->entry_speed_sqr = min( current->max_entry_speed_sqr, 2*current->acceleration*current->millimeters);
      
    // Reverse Pass: Coarsely maximize all possible deceleration curves back-planning from the last
    // block in buffer. Cease planning when: (1) the last optimal planned pointer is reached.
    // (2) the safe block pointer is reached, whereby the planned pointer is updated.
    // NOTE: Forward pass will later refine and correct the reverse pass to create an optimal plan.
    // NOTE: If the safe block is encountered before the planned block pointer, we know the safe block
    // will be recomputed within the plan. So, we need to update it if it is partially completed.
    float entry_speed_sqr;
    plan_block_t *next;
    block_index = prev_block_index(block_index);

    if (block_index == block_buffer_safe) { // !! OR plan pointer? Yes I think so.
      
      // Only two plannable blocks in buffer. Compute previous block based on 
      // !!! May only work if a new block is being added. Not for an override. The exit speed isn't zero.
      // !!! Need to make the current entry speed calculation after this.
      plan_update_partial_block(block_index, 0.0);
      block_buffer_planned = block_index;
printString("y");
    
    } else {
      
      // Three or more plan-able 
      while (block_index != block_buffer_planned) { 

        next = current;
        current = &block_buffer[block_index];

        // Increment block index early to check if the safe block is before the current block. If encountered,
        // this is an exit condition as we can't go further than this block in the reverse pass.
        block_index = prev_block_index(block_index);
        if (block_index == block_buffer_safe) {
          // Check if the safe block is partially completed. If so, update it before its exit speed 
          // (=current->entry speed) is over-written.
          // TODO: The update breaks with feedrate overrides, because the replanning process no longer has
          // the previous nominal speed to update this block with. There will need to be something along the
          // lines of a nominal speed change check and send the correct value to this function.
          plan_update_partial_block(block_index,current->entry_speed_sqr);
printString("x");
          // Set planned pointer at safe block and for loop exit after following computation is done.
          block_buffer_planned = block_index; 
        } 

        // Compute maximum entry speed decelerating over the current block from its exit speed.
        if (current->entry_speed_sqr != current->max_entry_speed_sqr) {
          entry_speed_sqr = next->entry_speed_sqr + 2*current->acceleration*current->millimeters;
          if (entry_speed_sqr < current->max_entry_speed_sqr) {
            current->entry_speed_sqr = entry_speed_sqr;
          } else {
            current->entry_speed_sqr = current->max_entry_speed_sqr;
          }
        }
      }
      
    }    

    // Forward Pass: Forward plan the acceleration curve from the planned pointer onward.
    // Also scans for optimal plan breakpoints and appropriately updates the planned pointer.
    next = &block_buffer[block_buffer_planned]; // Begin at buffer planned pointer
    block_index = next_block_index(block_buffer_planned); 
    while (block_index != next_buffer_head) {
      current = next;
      next = &block_buffer[block_index];
      
      // Any acceleration detected in the forward pass automatically moves the optimal planned
      // pointer forward, since everything before this is all optimal. In other words, nothing
      // can improve the plan from the buffer tail to the planned pointer by logic.
      if (current->entry_speed_sqr < next->entry_speed_sqr) {
        entry_speed_sqr = current->entry_speed_sqr + 2*current->acceleration*current->millimeters;
        // If true, current block is full-acceleration and we can move the planned pointer forward.
        if (entry_speed_sqr < next->entry_speed_sqr) {
          next->entry_speed_sqr = entry_speed_sqr; // Always <= max_entry_speed_sqr. Backward pass sets this.
          block_buffer_planned = block_index; // Set optimal plan pointer.
        }
      }
      
      // Any block set at its maximum entry speed also creates an optimal plan up to this
      // point in the buffer. When the plan is bracketed by either the beginning of the
      // buffer and a maximum entry speed or two maximum entry speeds, every block in between
      // cannot logically be further improved. Hence, we don't have to recompute them anymore.
      if (next->entry_speed_sqr == next->max_entry_speed_sqr) {
        block_buffer_planned = block_index; // Set optimal plan pointer
      }
      
      block_index = next_block_index( block_index );
    }
    
  }  
  
}


void plan_reset_buffer()
{
  block_buffer_planned = block_buffer_tail;
}

void plan_init() 
{
  block_buffer_tail = 0;
  block_buffer_head = 0; // Empty = tail
  next_buffer_head = 1; // next_block_index(block_buffer_head)
  plan_reset_buffer();
  memset(&pl, 0, sizeof(pl)); // Clear planner struct
}


void plan_discard_current_block() 
{
  if (block_buffer_head != block_buffer_tail) { // Discard non-empty buffer.
    block_buffer_tail = next_block_index( block_buffer_tail );
  }
}


plan_block_t *plan_get_current_block() 
{
  if (block_buffer_head == block_buffer_tail) { // Buffer empty
    plan_reset_buffer();
    return(NULL); 
  } 
  return(&block_buffer[block_buffer_tail]);
}


plan_block_t *plan_get_block_by_index(uint8_t block_index) 
{
  if (block_buffer_head == block_index) { return(NULL); }
  return(&block_buffer[block_index]);
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


// Add a new linear movement to the buffer. target[N_AXIS] is the signed, absolute target position
// in millimeters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
// rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
// All position data passed to the planner must be in terms of machine position to keep the planner 
// independent of any coordinate system changes and offsets, which are handled by the g-code parser.
// NOTE: Assumes buffer is available. Buffer checks are handled at a higher level by motion_control.
// In other words, the buffer head is never equal to the buffer tail.  Also the feed rate input value
// is used in three ways: as a normal feed rate if invert_feed_rate is false, as inverse time if
// invert_feed_rate is true, or as seek/rapids rate if the feed_rate value is negative (and
// invert_feed_rate always false).
void plan_buffer_line(float *target, float feed_rate, uint8_t invert_feed_rate) 
{
  // Prepare and initialize new block
  plan_block_t *block = &block_buffer[block_buffer_head];
  block->step_event_count = 0;
  block->millimeters = 0;
  block->direction_bits = 0;
  block->acceleration = SOME_LARGE_VALUE; // Scaled down to maximum acceleration later

  // Compute and store initial move distance data.
  // TODO: After this for-loop, we don't touch the stepper algorithm data. Might be a good idea
  // to try to keep these types of things completely separate from the planner for portability.
  int32_t target_steps[N_AXIS];
  float unit_vec[N_AXIS], delta_mm;
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    // Calculate target position in absolute steps. This conversion should be consistent throughout.
    target_steps[idx] = lround(target[idx]*settings.steps_per_mm[idx]);
  
    // Number of steps for each axis and determine max step events
    block->steps[idx] = labs(target_steps[idx]-pl.position[idx]);
    block->step_event_count = max(block->step_event_count, block->steps[idx]);
    
    // Compute individual axes distance for move and prep unit vector calculations.
    // NOTE: Computes true distance from converted step values.
    delta_mm = (target_steps[idx] - pl.position[idx])/settings.steps_per_mm[idx];
    unit_vec[idx] = delta_mm; // Store unit vector numerator. Denominator computed later.
        
    // Set direction bits. Bit enabled always means direction is negative.
    if (delta_mm < 0 ) { block->direction_bits |= get_direction_mask(idx); }
    
    // Incrementally compute total move distance by Euclidean norm. First add square of each term.
    block->millimeters += delta_mm*delta_mm;
  }
  block->millimeters = sqrt(block->millimeters); // Complete millimeters calculation with sqrt()
  
  // Bail if this is a zero-length block. Highly unlikely to occur.
  if (block->step_event_count == 0) { return; } 
  
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
  float inverse_unit_vec_value;
  float inverse_millimeters = 1.0/block->millimeters;  // Inverse millimeters to remove multiple float divides	
  float junction_cos_theta = 0;
  for (idx=0; idx<N_AXIS; idx++) {
    if (unit_vec[idx] != 0) {  // Avoid divide by zero.
      unit_vec[idx] *= inverse_millimeters;  // Complete unit vector calculation
      inverse_unit_vec_value = abs(1.0/unit_vec[idx]); // Inverse to remove multiple float divides.

      // Check and limit feed rate against max individual axis velocities and accelerations
      feed_rate = min(feed_rate,settings.max_velocity[idx]*inverse_unit_vec_value);
      block->acceleration = min(block->acceleration,settings.acceleration[idx]*inverse_unit_vec_value);

      // Incrementally compute cosine of angle between previous and current path. Cos(theta) of the junction
      // between the current move and the previous move is simply the dot product of the two unit vectors, 
      // where prev_unit_vec is negative. Used later to compute maximum junction speed.
      junction_cos_theta -= pl.previous_unit_vec[idx] * unit_vec[idx];
    }
  }


  // TODO: Need to check this method handling zero junction speeds when starting from rest.
  if (block_buffer_head == block_buffer_tail) {
  
    // Initialize block entry speed as zero. Assume it will be starting from rest. Planner will correct this later.
    // !!! Ensures when the first block starts from zero speed. If we do this in the planner, this will break
    // feedrate overrides later, as you can override this single block and it maybe moving already at a given rate.
    // Better to do it here and make it clean.
    // !!! Shouldn't need this for anything other than a single block.
    block->entry_speed_sqr = 0.0;
    block->max_junction_speed_sqr = 0.0; // Starting from rest. Enforce start from zero velocity.
  
  } else {
    /* 
       Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
       Let a circle be tangent to both previous and current path line segments, where the junction 
       deviation is defined as the distance from the junction to the closest edge of the circle, 
       colinear with the circle center. The circular segment joining the two paths represents the 
       path of centripetal acceleration. Solve for max velocity based on max acceleration about the
       radius of the circle, defined indirectly by junction deviation. This may be also viewed as 
       path width or max_jerk in the previous grbl version. This approach does not actually deviate 
       from path, but used as a robust way to compute cornering speeds, as it takes into account the
       nonlinearities of both the junction angle and junction velocity.

       NOTE: If the junction deviation value is finite, Grbl executes the motions in an exact path 
       mode (G61). If the junction deviation value is zero, Grbl will execute the motion in an exact
       stop mode (G61.1) manner. In the future, if continuous mode (G64) is desired, the math here
       is exactly the same. Instead of motioning all the way to junction point, the machine will
       just follow the arc circle defined here. The Arduino doesn't have the CPU cycles to perform
       a continuous mode path, but ARM-based microcontrollers most certainly do. 
       
       NOTE: The max junction speed is a fixed value, since machine acceleration limits cannot be
       changed dynamically during operation nor can the line segment geometry. This must be kept in
       memory in the event of a feedrate override changing the nominal speeds of blocks, which can 
       change the overall maximum entry speed conditions of all blocks.
       
    */
    // NOTE: Computed without any expensive trig, sin() or acos(), by trig half angle identity of cos(theta).
    float sin_theta_d2 = sqrt(0.5*(1.0-junction_cos_theta)); // Trig half angle identity. Always positive.

    // TODO: Acceleration used in calculation needs to be limited by the minimum of the two junctions. 
    block->max_junction_speed_sqr = max( MINIMUM_JUNCTION_SPEED*MINIMUM_JUNCTION_SPEED,
                                 (block->acceleration * settings.junction_deviation * sin_theta_d2)/(1.0-sin_theta_d2) );
  }

  // Store block nominal speed
  block->nominal_speed_sqr = feed_rate*feed_rate; // (mm/min). Always > 0
  
  // Compute the junction maximum entry based on the minimum of the junction speed and neighboring nominal speeds.
  // TODO: Should call a function to determine this. The function can be used elsewhere for feedrate overrides later.
  block->max_entry_speed_sqr = min(block->max_junction_speed_sqr, 
                                   min(block->nominal_speed_sqr,pl.previous_nominal_speed_sqr));
  
  // Update previous path unit_vector and nominal speed (squared)
  memcpy(pl.previous_unit_vec, unit_vec, sizeof(unit_vec)); // pl.previous_unit_vec[] = unit_vec[]
  pl.previous_nominal_speed_sqr = block->nominal_speed_sqr;
    
  // Update planner position
  memcpy(pl.position, target_steps, sizeof(target_steps)); // pl.position[] = target_steps[]

  planner_recalculate(); 

  // Update buffer head and next buffer head indices. Advance only after new plan has been computed.
  block_buffer_head = next_buffer_head;  
  next_buffer_head = next_block_index(block_buffer_head);
  


int32_t blength = block_buffer_head - block_buffer_tail;
if (blength < 0) { blength += BLOCK_BUFFER_SIZE; } 
printInteger(blength);


}


// Reset the planner position vectors. Called by the system abort/initialization routine.
void plan_sync_position()
{
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    pl.position[idx] = sys.position[idx];
  }
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
                                                        
  Calculates the "trapezoid" velocity profile parameters of a planner block for the stepper 
  algorithm. The planner computes the entry and exit speeds of each block, but does not bother to 
  determine the details of the velocity profiles within them, as they aren't needed for computing 
  an optimal plan. When the stepper algorithm begins to execute a block, the block velocity profiles 
  are computed ad hoc. 
  
  Each block velocity profiles can be described as either a trapezoidal or a triangular shape. The
  trapezoid occurs when the block reaches the nominal speed of the block and cruises for a period of
  time. A triangle occurs when the nominal speed is not reached within the block. Both of these 
  velocity profiles may also be truncated on either end with no acceleration or deceleration ramps, 
  as they can be influenced by the conditions of neighboring blocks.
  
  The following function determines the type of velocity profile and stores the minimum required
  information for the stepper algorithm to execute the calculated profiles. Since the stepper 
  algorithm always assumes to begin accelerating from the initial_rate and cruise if the nominal_rate
  is reached, we only need to know when to begin deceleration to the end of the block. Hence, only
  the distance from the end of the block to begin a deceleration ramp are computed.
*/
float plan_calculate_velocity_profile(uint8_t block_index) 
{  
  plan_block_t *current_block = &block_buffer[block_index];
  
  // Determine current block exit speed
  float exit_speed_sqr = 0.0; // Initialize for end of planner buffer. Zero speed.
  plan_block_t *next_block = plan_get_block_by_index(next_block_index(block_index));
  if (next_block != NULL) { exit_speed_sqr = next_block->entry_speed_sqr; } // Exit speed is the entry speed of next buffer block
  
  // First determine intersection distance (in steps) from the exit point for a triangular profile.
  // Computes: d_intersect = distance/2 + (v_entry^2-v_exit^2)/(4*acceleration)
  float intersect_distance = 0.5*( current_block->millimeters + (current_block->entry_speed_sqr-exit_speed_sqr)/(2*current_block->acceleration) );  
  
  // Check if this is a pure acceleration block by a intersection distance less than zero. Also
  // prevents signed and unsigned integer conversion errors.
  if (intersect_distance > 0 ) {
    float decelerate_distance;
    // Determine deceleration distance (in steps) from nominal speed to exit speed for a trapezoidal profile.
    // Value is never negative. Nominal speed is always greater than or equal to the exit speed.
    // Computes: d_decelerate = (v_nominal^2 - v_exit^2)/(2*acceleration)
    decelerate_distance = (current_block->nominal_speed_sqr - exit_speed_sqr)/(2*current_block->acceleration);

    // The lesser of the two triangle and trapezoid distances always defines the velocity profile.
    if (decelerate_distance > intersect_distance) { decelerate_distance = intersect_distance; }
    
    // Finally, check if this is a pure deceleration block.
    if (decelerate_distance > current_block->millimeters) { return(0.0); }
    else { return( (current_block->millimeters-decelerate_distance) ); }
  }
  return( current_block->millimeters ); // No deceleration in velocity profile.
}        


// Re-initialize buffer plan with a partially completed block, assumed to exist at the buffer tail.
// Called after a steppers have come to a complete stop for a feed hold and the cycle is stopped.
void plan_cycle_reinitialize(int32_t step_events_remaining) 
{
  plan_block_t *block = &block_buffer[block_buffer_tail]; // Point to partially completed block
  
  // Only remaining millimeters and step_event_count need to be updated for planner recalculate. 
  // Other variables (step_x, step_y, step_z, rate_delta, etc.) all need to remain the same to
  // ensure the original planned motion is resumed exactly.
  block->millimeters = (block->millimeters*step_events_remaining)/block->step_event_count;
  block->step_event_count = step_events_remaining;
  
  // Re-plan from a complete stop. Reset planner entry speeds and buffer planned pointer.
  block->entry_speed_sqr = 0.0;
  block->max_entry_speed_sqr = 0.0;
  block_buffer_planned = block_buffer_tail;
  planner_recalculate();  
}
