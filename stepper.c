/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
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

#include <avr/interrupt.h>
#include "stepper.h"
#include "config.h"
#include "settings.h"
#include "planner.h"
#include "nuts_bolts.h"


// Some useful constants
#define TICKS_PER_MICROSECOND (F_CPU/1000000)
#define DT_SEGMENT (1.0/(ACCELERATION_TICKS_PER_SECOND*60.0)) // min/segment

#define RAMP_ACCEL 0
#define RAMP_CRUISE 1
#define RAMP_DECEL 2


// Stores the planner block Bresenham algorithm execution data for the segments in the segment 
// buffer. Normally, this buffer is partially in-use, but, for the worst case scenario, it will
// never exceed the number of accessible stepper buffer segments (SEGMENT_BUFFER_SIZE-1).
// NOTE: This data is copied from the prepped planner blocks so that the planner blocks may be
// discarded when entirely consumed and completed by the segment buffer.
typedef struct {  
  uint8_t direction_bits;
  int32_t steps[N_AXIS];
  int32_t step_event_count;
} st_block_t;
static st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE-1];
// TODO: Directly adjust this parameters to stop motion of individual axes for the homing cycle.
// But this may require this to be volatile if it is controlled by an interrupt.

// Primary stepper segment ring buffer. Contains small, short line segments for the stepper 
// algorithm to execute, which are "checked-out" incrementally from the first block in the
// planner buffer. Once "checked-out", the steps in the segments buffer cannot be modified by 
// the planner, where the remaining planner block steps still can.
typedef struct {
  uint8_t n_step;         // Number of step events to be executed for this segment
  uint8_t st_block_index; // Stepper block data index. Uses this information to execute this segment.
  int32_t phase_dist;     // Remaining step fraction to tick before completing segment.
  int32_t dist_per_tick;  // Step distance traveled per ISR tick, aka step rate.
} segment_t;
static segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

// Stepper state variable. Contains running data and trapezoid variables.
typedef struct {
  // Used by the bresenham line algorithm
  int32_t counter_x,        // Counter variables for the bresenham line tracer
          counter_y, 
          counter_z;

  // Used by inverse time algorithm to track step rate
  int32_t counter_dist;     // Inverse time distance traveled since last step event
  
  // Used by the stepper driver interrupt
  uint8_t execute_step;     // Flags step execution for each interrupt.
  uint8_t step_pulse_time;  // Step pulse reset time after step rise
  uint8_t out_bits;         // The next stepping-bits to be output

  uint8_t step_count;       // Steps remaining in line segment motion  
  uint8_t exec_block_index; // Tracks the current st_block index. Change indicates new block.
  st_block_t *exec_block;   // Pointer to the block data for the segment being executed
  segment_t *exec_segment;  // Pointer to the segment being executed
} stepper_t;
static stepper_t st;

// Step segment ring buffer indices
static volatile uint8_t segment_buffer_tail;
static uint8_t segment_buffer_head;
static uint8_t segment_next_head;

// Used to avoid ISR nesting of the "Stepper Driver Interrupt". Should never occur though.
static volatile uint8_t busy;   

// Pointers for the step segment being prepped from the planner buffer. Accessed only by the
// main program. Pointers may be planning segments or planner blocks ahead of what being executed.
static plan_block_t *pl_block;     // Pointer to the planner block being prepped
static st_block_t *st_prep_block;  // Pointer to the stepper block data being prepped 

// Segment preparation data struct. Contains all the necessary information to compute new segments
// based on the current executing planner block.
typedef struct {
  uint8_t st_block_index;  // Index of stepper common data block being prepped
  uint8_t flag_partial_block;  // Flag indicating the last block completed. Time to load a new one.

  float step_per_mm;           // Current planner block step/millimeter conversion scalar
  float steps_remaining;
  int32_t step_events_remaining; // Tracks step event count for the executing planner block
  
  uint8_t ramp_type;      // Current segment ramp state
  float millimeters_remaining;
  float current_speed;    // Current speed at the end of the segment buffer (mm/min)
  float maximum_speed;    // Maximum speed of executing block. Not always nominal speed. (mm/min)
  float exit_speed;       // Exit speed of executing block (mm/min)
  float accelerate_until; // Acceleration ramp end measured from end of block (mm)
  float decelerate_after; // Deceleration ramp start measured from end of block (mm)
} st_prep_t;
static st_prep_t prep;


/*    BLOCK VELOCITY PROFILE DEFINITION 
          __________________________
         /|                        |\     _________________         ^
        / |                        | \   /|               |\        |
       /  |                        |  \ / |               | \       s
      /   |                        |   |  |               |  \      p
     /    |                        |   |  |               |   \     e
    +-----+------------------------+---+--+---------------+----+    e
    |               BLOCK 1            ^      BLOCK 2          |    d
                                       |
                  time ----->      EXAMPLE: Block 2 entry speed is at max junction velocity
  
  The planner block buffer is planned assuming constant acceleration velocity profiles and are
  continuously joined at block junctions as shown above. However, the planner only actively computes
  the block entry speeds for an optimal velocity plan, but does not compute the block internal
  velocity profiles. These velocity profiles are computed ad-hoc as they are executed by the 
  stepper algorithm and consists of only 7 possible types of profiles: cruise-only, cruise-
  deceleration, acceleration-cruise, acceleration-only, deceleration-only, full-trapezoid, and 
  triangle(no cruise).

                                        maximum_speed (< nominal_speed) ->  + 
                    +--------+ <- maximum_speed (= nominal_speed)          /|\                                         
                   /          \                                           / | \                      
 current_speed -> +            \                                         /  |  + <- exit_speed
                  |             + <- exit_speed                         /   |  |                       
                  +-------------+                     current_speed -> +----+--+                   
                   time -->  ^  ^                                           ^  ^                       
                             |  |                                           |  |                       
                decelerate_after(in mm)                             decelerate_after(in mm)
                    ^           ^                                           ^  ^
                    |           |                                           |  |
                accelerate_until(in mm)                             accelerate_until(in mm)
                    
  The step segment buffer computes the executing block velocity profile and tracks the critical
  parameters for the stepper algorithm to accurately trace the profile. These critical parameters 
  are shown and defined in the above illustration.
*/

// Stepper state initialization. Cycle should only start if the st.cycle_start flag is
// enabled. Startup init and limits call this function but shouldn't start the cycle.
void st_wake_up() 
{
  // Enable steppers by resetting the stepper disable port
  if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) { 
    STEPPERS_DISABLE_PORT |= (1<<STEPPERS_DISABLE_BIT); 
  } else { 
    STEPPERS_DISABLE_PORT &= ~(1<<STEPPERS_DISABLE_BIT);
  }
  if (sys.state == STATE_CYCLE) {
    // Initialize stepper output bits
    st.out_bits = settings.invert_mask; 
    // Initialize step pulse timing from settings.
    st.step_pulse_time = -(((settings.pulse_microseconds-2)*TICKS_PER_MICROSECOND) >> 3);
    
    // Enable stepper driver interrupt
    TCNT2 = 0; // Clear Timer2
    TIMSK2 |= (1<<OCIE2A); // Enable Timer2 Compare Match A interrupt
    TCCR2B = (1<<CS21); // Begin Timer2. Full speed, 1/8 prescaler
  }
}


// Stepper shutdown
void st_go_idle() 
{
  // Disable stepper driver interrupt. Allow Timer0 to finish. It will disable itself.
  TIMSK2 &= ~(1<<OCIE2A); // Disable Timer2 interrupt
  TCCR2B = 0; // Disable Timer2
  busy = false;

  // Disable steppers only upon system alarm activated or by user setting to not be kept enabled.
  if ((settings.stepper_idle_lock_time != 0xff) || bit_istrue(sys.execute,EXEC_ALARM)) {
    // Force stepper dwell to lock axes for a defined amount of time to ensure the axes come to a complete
    // stop and not drift from residual inertial forces at the end of the last movement.
    delay_ms(settings.stepper_idle_lock_time);
    if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) { 
      STEPPERS_DISABLE_PORT &= ~(1<<STEPPERS_DISABLE_BIT); 
    } else { 
      STEPPERS_DISABLE_PORT |= (1<<STEPPERS_DISABLE_BIT); 
    }   
  }
}


/* "The Stepper Driver Interrupt" - This timer interrupt is the workhorse of Grbl. It is based
   on an inverse time stepper algorithm, where a timer ticks at a constant frequency and uses 
   time-distance counters to track when its the approximate time for a step event. For reference,
   a similar inverse-time algorithm by Pramod Ranade is susceptible to numerical round-off, as
   described, meaning that some axes steps may not execute correctly for a given multi-axis motion.
     Grbl's algorithm differs by using a single inverse time-distance counter to manage a
   Bresenham line algorithm for multi-axis step events, which ensures the number of steps for
   each axis are executed exactly. In other words, Grbl uses a Bresenham within a Bresenham 
   algorithm, where one tracks time for step events and the other steps for multi-axis moves.
   Grbl specifically uses the Bresenham algorithm due to its innate mathematical exactness and
   low computational overhead, requiring simple integer +,- counters only.
     This interrupt pops blocks from the step segment buffer and executes them by pulsing the 
   stepper pins appropriately. It is supported by The Stepper Port Reset Interrupt which it uses 
   to reset the stepper port after each pulse. The bresenham line tracer algorithm controls all 
   three stepper outputs simultaneously with these two interrupts.
*/
/* TODO: 
   - Measured time in ISR. Typical and worst-case. Roughly 5usec min to 25usec max. Good enough.
     There are no major changes to the base operations of this ISR with the new segment buffer.
   - Determine if placing the position counters elsewhere (or change them to 8-bit variables that
     are added to the system position counters at the end of a segment) frees up cycles.
   - Create NOTE: to describe that the total time in this ISR must be less than the ISR frequency
     in its worst case scenario.
*/
ISR(TIMER2_COMPA_vect)
{
// SPINDLE_ENABLE_PORT ^= 1<<SPINDLE_ENABLE_BIT; // Debug: Used to time ISR
  if (busy) { return; } // The busy-flag is used to avoid reentering this interrupt
  
  // Pulse stepper port pins, if flagged. New block dir will always be set one timer tick 
  // before any step pulse due to algorithm design.
  if (st.execute_step) {
    st.execute_step = false;
    STEPPING_PORT = ( STEPPING_PORT & ~(DIRECTION_MASK | STEP_MASK) ) | st.out_bits;
    TCNT0 = st.step_pulse_time; // Reload Timer0 counter.
    TCCR0B = (1<<CS21); // Begin Timer0. Full speed, 1/8 prescaler
  }
  
  busy = true;
  sei(); // Re-enable interrupts to allow Stepper Port Reset Interrupt to fire on-time. 
         // NOTE: The remaining code in this ISR will finish before returning to main program.
    
  // If there is no step segment, attempt to pop one from the stepper buffer
  if (st.exec_segment == NULL) {
    
    // Anything in the buffer? If so, load and initialize next step segment.
    if (segment_buffer_head != segment_buffer_tail) {
      
      // Initialize new step segment and load number of steps to execute
      st.exec_segment = &segment_buffer[segment_buffer_tail];
      st.step_count = st.exec_segment->n_step;
    
      // If the new segment starts a new planner block, initialize stepper variables and counters.
      // NOTE: When the segment data index changes, this indicates a new planner block.
      if ( st.exec_block_index != st.exec_segment->st_block_index ) {
        st.exec_block_index = st.exec_segment->st_block_index;
        st.exec_block = &st_block_buffer[st.exec_block_index];
        
        // Initialize direction bits for block. Set execute flag to set directions bits upon next ISR tick.
        st.out_bits = st.exec_block->direction_bits ^ settings.invert_mask;
        st.execute_step = true;
        
        // Initialize Bresenham line and distance counters
        st.counter_x = (st.exec_block->step_event_count >> 1);
        st.counter_y = st.counter_x;
        st.counter_z = st.counter_x;
        st.counter_dist = 0;         
      }
      
    } else {
      // Segment buffer empty. Shutdown.
      st_go_idle();
      bit_true(sys.execute,EXEC_CYCLE_STOP); // Flag main program for cycle end
      return; // Nothing to do but exit.
    }  
    
  } 
    
  // Iterate inverse time counter. Triggers each Bresenham step event.
  st.counter_dist += st.exec_segment->dist_per_tick; 
  
  // Execute Bresenham step event, when it's time to do so.
  if (st.counter_dist > INV_TIME_MULTIPLIER) {
    if (st.step_count > 0) {  // Block phase correction from executing step. 
      st.counter_dist -= INV_TIME_MULTIPLIER; // Reload inverse time counter
      st.step_count--; // Decrement step events count    

      // Execute step displacement profile by Bresenham line algorithm
      st.execute_step = true;
      st.out_bits = st.exec_block->direction_bits; // Reset out_bits and reload direction bits      
      st.counter_x -= st.exec_block->steps[X_AXIS];
      if (st.counter_x < 0) {
        st.out_bits |= (1<<X_STEP_BIT);
        st.counter_x += st.exec_block->step_event_count;
        if (st.out_bits & (1<<X_DIRECTION_BIT)) { sys.position[X_AXIS]--; }
        else { sys.position[X_AXIS]++; }
      }
      st.counter_y -= st.exec_block->steps[Y_AXIS];
      if (st.counter_y < 0) {
        st.out_bits |= (1<<Y_STEP_BIT);
        st.counter_y += st.exec_block->step_event_count;
        if (st.out_bits & (1<<Y_DIRECTION_BIT)) { sys.position[Y_AXIS]--; }
        else { sys.position[Y_AXIS]++; }
      }
      st.counter_z -= st.exec_block->steps[Z_AXIS];
      if (st.counter_z < 0) {
        st.out_bits |= (1<<Z_STEP_BIT);
        st.counter_z += st.exec_block->step_event_count;
        if (st.out_bits & (1<<Z_DIRECTION_BIT)) { sys.position[Z_AXIS]--; }
        else { sys.position[Z_AXIS]++; }
      }  
      st.out_bits ^= settings.invert_mask;  // Apply step port invert mask    
    }
  }
  
  if (st.step_count == 0) {
    if (st.counter_dist > st.exec_segment->phase_dist) { 
      // Segment is complete. Discard current segment and advance segment indexing.
      st.exec_segment = NULL;
      if ( ++segment_buffer_tail == SEGMENT_BUFFER_SIZE) { segment_buffer_tail = 0; }
    }
  }
  
  busy = false;
// SPINDLE_ENABLE_PORT ^= 1<<SPINDLE_ENABLE_BIT;  
}


/* The Stepper Port Reset Interrupt: Timer0 OVF interrupt handles the falling edge of the step
   pulse. This should always trigger before the next Timer2 COMPA interrupt and independently
   finish, if Timer2 is disabled after completing a move. */
ISR(TIMER0_OVF_vect)
{
  STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | (settings.invert_mask & STEP_MASK); 
  TCCR0B = 0; // Disable timer until needed.
}


// Reset and clear stepper subsystem variables
void st_reset()
{
  memset(&prep, 0, sizeof(prep));
  memset(&st, 0, sizeof(st));
  st.exec_segment = NULL;
  pl_block = NULL;  // Planner block pointer used by segment buffer

  segment_buffer_tail = 0;
  segment_buffer_head = 0; // empty = tail
  segment_next_head = 1;
  busy = false;
}


// Initialize and start the stepper motor subsystem
void st_init()
{
  // Configure directions of interface pins
  STEPPING_DDR |= STEPPING_MASK;
  STEPPING_PORT = (STEPPING_PORT & ~STEPPING_MASK) | settings.invert_mask;
  STEPPERS_DISABLE_DDR |= 1<<STEPPERS_DISABLE_BIT;
	
  // Configure Timer 2
  TIMSK2 &= ~(1<<OCIE2A); // Disable Timer2 interrupt while configuring it
  TCCR2B = 0; // Disable Timer2 until needed
  TCNT2 = 0; // Clear Timer2 counter
  TCCR2A = (1<<WGM21);  // Set CTC mode
  OCR2A = (F_CPU/ISR_TICKS_PER_SECOND)/8 - 1; // Set Timer2 CTC rate

  // Configure Timer 0
  TIMSK0 &= ~(1<<TOIE0);
  TCCR0A = 0; // Normal operation
  TCCR0B = 0; // Disable Timer0 until needed
  TIMSK0 |= (1<<TOIE0); // Enable overflow interrupt
  
  // Start in the idle state, but first wake up to check for keep steppers enabled option.
  st_wake_up();
  st_go_idle();
}


// Planner external interface to start stepper interrupt and execute the blocks in queue. Called
// by the main program functions: planner auto-start and run-time command execution.
void st_cycle_start() 
{
  if (sys.state == STATE_QUEUED) {
    sys.state = STATE_CYCLE;
    st_prep_buffer(); // Initialize step segment buffer before beginning cycle.
    st_wake_up();
  }
}


// Execute a feed hold with deceleration, only during cycle. Called by main program.
void st_feed_hold() 
{
  if (sys.state == STATE_CYCLE) {
    sys.state = STATE_HOLD;
    st_update_plan_block_parameters();
    st_prep_buffer();
    sys.auto_start = false; // Disable planner auto start upon feed hold.
  }
}


// Reinitializes the cycle plan and stepper system after a feed hold for a resume. Called by 
// runtime command execution in the main program, ensuring that the planner re-plans safely.
// NOTE: Bresenham algorithm variables are still maintained through both the planner and stepper
// cycle reinitializations. The stepper path should continue exactly as if nothing has happened.
// Only the planner de/ac-celerations profiles and stepper rates have been updated.
void st_cycle_reinitialize()
{
  if (sys.state != STATE_QUEUED) {    
    sys.state = STATE_IDLE;
  }
}


// Called by planner_recalculate() when the executing block is updated by the new plan.
void st_update_plan_block_parameters()
{ 
  if (pl_block != NULL) { // Ignore if at start of a new block.
    prep.flag_partial_block = true;
    pl_block->millimeters = prep.millimeters_remaining;
    pl_block->entry_speed_sqr = prep.current_speed*prep.current_speed; // Update entry speed.
    pl_block = NULL; // Flag st_prep_segment() to load new velocity profile.
  }
}


/* Prepares step segment buffer. Continuously called from main program. 

   The segment buffer is an intermediary buffer interface between the execution of steps
   by the stepper algorithm and the velocity profiles generated by the planner. The stepper
   algorithm only executes steps within the segment buffer and is filled by the main program
   when steps are "checked-out" from the first block in the planner buffer. This keeps the
   step execution and planning optimization processes atomic and protected from each other.
   The number of steps "checked-out" from the planner buffer and the number of segments in
   the segment buffer is sized and computed such that no operation in the main program takes
   longer than the time it takes the stepper algorithm to empty it before refilling it. 
   Currently, the segment buffer conservatively holds roughly up to 40-50 msec of steps. 

   NOTE: The segment buffer executes a computed number of steps over a configured segment 
   execution time period, except at an end of a planner block where the segment execution
   gets truncated by the lack of travel distance. Since steps are integer values and to keep
   the distance traveled over the segment exact, a fractional step remaining after the last
   executed step in a segment is handled by allowing the stepper algorithm distance counters
   to tick to this fractional value without triggering a full step. So, when the next segment
   is loaded for execution, its first full step will already have the distance counters primed
   with the previous segment fractional step and will execute exactly on time according to 
   the planner block velocity profile. This ensures the step phasing between segments are kept
   in sync and prevents artificially created accelerations between segments if they are not
   accounted for. This allows the stepper algorithm to run at very high step rates without
   losing steps.
*/
/* 
   TODO: With feedrate overrides, increases to the override value will not significantly
     change the planner and stepper current operation. When the value increases, we simply
     need to recompute the block plan with new nominal speeds and maximum junction velocities.
     However with a decreasing feedrate override, this gets a little tricky. The current block
     plan is optimal, so if we try to reduce the feed rates, it may be impossible to create 
     a feasible plan at its current operating speed and decelerate down to zero at the end of
     the buffer. We first have to enforce a deceleration to meet and intersect with the reduced
     feedrate override plan. For example, if the current block is cruising at a nominal rate
     and the feedrate override is reduced, the new nominal rate will now be lower. The velocity
     profile must first decelerate to the new nominal rate and then follow on the new plan. 
        Another issue is whether or not a feedrate override reduction causes a deceleration
     that acts over several planner blocks. For example, say that the plan is already heavily
     decelerating throughout it, reducing the feedrate override will not do much to it. So,
     how do we determine when to resume the new plan? One solution is to tie into the feed hold
     handling code to enforce a deceleration, but check when the current speed is less than or
     equal to the block maximum speed and is in an acceleration or cruising ramp. At this 
     point, we know that we can recompute the block velocity profile to meet and continue onto
     the new block plan.
*/
void st_prep_buffer()
{
  while (segment_buffer_tail != segment_next_head) { // Check if we need to fill the buffer.
    if (sys.state == STATE_QUEUED) { return; } // Block until a motion state is issued
    
    // Determine if we need to load a new planner block. If so, prepare step data.
    if (pl_block == NULL) {
      pl_block = plan_get_current_block(); // Query planner for a queued block
      if (pl_block == NULL) { return; } // No planner blocks. Exit.
                      
      // Check if the segment buffer completed the last planner block. If so, load the Bresenham
      // data for the block. If not, we are still mid-block and the velocity profile has changed.
      if (prep.flag_partial_block) {
        prep.flag_partial_block = false; // Reset flag
      } else {
        // Increment stepper common data index 
        if ( ++prep.st_block_index == (SEGMENT_BUFFER_SIZE-1) ) { prep.st_block_index = 0; }
        
        // Prepare and copy Bresenham algorithm segment data from the new planner block, so that
        // when the segment buffer completes the planner block, it may be discarded immediately.
        st_prep_block = &st_block_buffer[prep.st_block_index];
        st_prep_block->steps[X_AXIS] = pl_block->steps[X_AXIS];
        st_prep_block->steps[Y_AXIS] = pl_block->steps[Y_AXIS];
        st_prep_block->steps[Z_AXIS] = pl_block->steps[Z_AXIS];
        st_prep_block->direction_bits = pl_block->direction_bits;
        st_prep_block->step_event_count = pl_block->step_event_count;
        
        // Initialize segment buffer data for generating the segments.
        prep.step_events_remaining = st_prep_block->step_event_count;
        prep.steps_remaining = st_prep_block->step_event_count;
        prep.millimeters_remaining = pl_block->millimeters;
        prep.step_per_mm = prep.steps_remaining/prep.millimeters_remaining;

        if (sys.state == STATE_HOLD) { 
          prep.current_speed = prep.exit_speed; 
          pl_block->entry_speed_sqr = prep.exit_speed*prep.exit_speed; 
        }
        else { prep.current_speed = sqrt(pl_block->entry_speed_sqr); }
      }
     
      float inv_2_accel = 0.5/pl_block->acceleration;
      if (sys.state == STATE_HOLD) {
        // Compute velocity profile parameters for a feed hold in-progress.
        prep.ramp_type = RAMP_DECEL;
        float decel_dist = inv_2_accel*pl_block->entry_speed_sqr;
        if (decel_dist < prep.millimeters_remaining) {
          prep.exit_speed = 0.0;
          prep.steps_remaining = prep.step_per_mm*decel_dist;
          prep.millimeters_remaining = decel_dist;
        } else {
          prep.exit_speed = sqrt(pl_block->entry_speed_sqr-2*pl_block->acceleration*prep.millimeters_remaining);
        }
      } else {                
        // Compute velocity profile parameters of the prepped planner block.
        prep.ramp_type = RAMP_ACCEL; // Initialize as acceleration ramp.
        prep.accelerate_until = prep.millimeters_remaining; 
        prep.exit_speed = plan_get_exec_block_exit_speed();   
        float exit_speed_sqr = prep.exit_speed*prep.exit_speed;
        float intersect_distance =
                0.5*(prep.millimeters_remaining+inv_2_accel*(pl_block->entry_speed_sqr-exit_speed_sqr));
        if (intersect_distance > 0.0) {
          if (intersect_distance < prep.millimeters_remaining) { // Either trapezoid or triangle types
            // NOTE: For acceleration-cruise and cruise-only types, following calculation will be 0.0.
            prep.decelerate_after = inv_2_accel*(pl_block->nominal_speed_sqr-exit_speed_sqr);
            if (prep.decelerate_after < intersect_distance) { // Trapezoid type
              prep.maximum_speed = sqrt(pl_block->nominal_speed_sqr);
              if (pl_block->entry_speed_sqr == pl_block->nominal_speed_sqr) { 
                // Cruise-deceleration or cruise-only type.
                prep.ramp_type = RAMP_CRUISE;
              } else {
                // Full-trapezoid or acceleration-cruise types
                prep.accelerate_until -= inv_2_accel*(pl_block->nominal_speed_sqr-pl_block->entry_speed_sqr); 
              }
            } else { // Triangle type
              prep.accelerate_until = intersect_distance;
              prep.decelerate_after = intersect_distance;
              prep.maximum_speed = sqrt(2.0*pl_block->acceleration*intersect_distance+exit_speed_sqr);
            }          
          } else { // Deceleration-only type
            prep.ramp_type = RAMP_DECEL;
            prep.decelerate_after = prep.millimeters_remaining;
            prep.maximum_speed = prep.current_speed;
          }
        } else { // Acceleration-only type
          prep.accelerate_until = 0.0;
          prep.decelerate_after = 0.0;
          prep.maximum_speed = prep.exit_speed;
        }
      }
      
    }

    // Initialize new segment
    segment_t *prep_segment = &segment_buffer[segment_buffer_head];

    // Set new segment to point to the current segment data block.
    prep_segment->st_block_index = prep.st_block_index;

    /* -----------------------------------------------------------------------------------
       Compute the average velocity of this new segment by determining the total distance
       traveled over the segment time DT_SEGMENT. The following code first attempts to create 
       a full segment based on the current ramp conditions. If the segment time is incomplete 
       when terminating at a ramp state change, the code will continue to loop through the
       progressing ramp states to fill the remaining segment execution time. However, if 
       an incomplete segment terminates at the end of the planner block, the segment is 
       considered completed despite having a truncated execution time less than DT_SEGMENT. 
    */   
    float dt = 0.0;
    float mm_remaining = prep.millimeters_remaining;
    float time_var = DT_SEGMENT; // Time worker variable
    float mm_var; // mm distance worker variable
    do {
      switch (prep.ramp_type) {
        case RAMP_ACCEL: 
          // NOTE: Acceleration ramp always computes during first loop only.
          mm_remaining -= DT_SEGMENT*(prep.current_speed + pl_block->acceleration*(0.5*DT_SEGMENT));
          if (mm_remaining < prep.accelerate_until) { // End of acceleration ramp.
            // Acceleration-cruise, acceleration-deceleration ramp junction, or end of block.
            mm_remaining = prep.accelerate_until; // NOTE: 0.0 at EOB
            time_var = 2.0*(prep.millimeters_remaining-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            if (mm_remaining == prep.decelerate_after) { prep.ramp_type = RAMP_DECEL; }
            else { prep.ramp_type = RAMP_CRUISE; }
            prep.current_speed = prep.maximum_speed;
          } else { // Acceleration only. 
            prep.current_speed += pl_block->acceleration*time_var;
          }
          break;
        case RAMP_CRUISE: 
          // NOTE: mm_var used to retain the last mm_remaining for incomplete segment time_var calculations.
          mm_var = mm_remaining - prep.maximum_speed*time_var;
          if (mm_var < prep.decelerate_after) { // End of cruise. 
            // Cruise-deceleration junction or end of block.
            time_var = (mm_remaining - prep.decelerate_after)/prep.maximum_speed;
            mm_remaining = prep.decelerate_after; // NOTE: 0.0 at EOB
            prep.ramp_type = RAMP_DECEL;
          } else { // Cruising only.         
            mm_remaining = mm_var; 
          } 
          break;
        default: // case RAMP_DECEL:
          // NOTE: mm_var used to catch negative decelerate distance values near zero speed.
          mm_var = time_var*(prep.current_speed - 0.5*pl_block->acceleration*time_var);
          if ((mm_var > 0.0) && (mm_var < mm_remaining)) { // Deceleration only.
            prep.current_speed -= pl_block->acceleration*time_var;
            // Check for near-zero speed and prevent divide by zero in rare scenarios.
            if (prep.current_speed > prep.exit_speed) { mm_remaining -= mm_var; }
            else { mm_remaining = 0.0; } // NOTE: Force EOB for now. May or may not be needed.
          } else { // End of block.
            time_var = 2.0*mm_remaining/(prep.current_speed+prep.exit_speed);
            mm_remaining = 0.0;
            // prep.current_speed = prep.exit_speed; // !! May be needed for feed hold reinitialization.
          }
      }
      dt += time_var; // Add computed ramp time to total segment time.
      if (dt < DT_SEGMENT) { time_var = DT_SEGMENT - dt; } // **Incomplete** At ramp junction.
      else { break; } // **Complete** Exit loop. Segment execution time maxed.      
    } while ( mm_remaining > 0.0 ); // **Complete** Exit loop. End of planner block.
   
    /* -----------------------------------------------------------------------------------
       Compute segment step rate, steps to execute, and step phase correction parameters.
       NOTE: Steps are computed by direct scalar conversion of the millimeter distance 
       remaining in the block, rather than incrementally tallying the steps executed per
       segment. This helps in removing floating point round-off issues of several additions. 
       However, since floats have only 7.2 significant digits, long moves with extremely 
       high step counts can exceed the precision of floats, which can lead to lost steps.
       Fortunately, this scenario is highly unlikely and unrealistic in CNC machines
       supported by Grbl (i.e. exceeding 10 meters axis travel at 200 step/mm).
    */
    // Use time_var to pre-compute dt inversion with integer multiplier.
    time_var = (INV_TIME_MULTIPLIER/(60.0*ISR_TICKS_PER_SECOND))/dt; // (mult/isr_tic)
    if (mm_remaining > 0.0) { 
      float steps_remaining = prep.step_per_mm*mm_remaining; 
      prep_segment->dist_per_tick = ceil( (prep.steps_remaining-steps_remaining)*time_var ); // (mult*step/isr_tic)

      // Compute number of steps to execute and segment step phase correction. 
      prep_segment->phase_dist = ceil(INV_TIME_MULTIPLIER*(ceil(steps_remaining)-steps_remaining));
      prep_segment->n_step = ceil(prep.steps_remaining)-ceil(steps_remaining);

      // Update step execution variables
      prep.step_events_remaining -= prep_segment->n_step;
      prep.millimeters_remaining = mm_remaining;      
      prep.steps_remaining = steps_remaining;                             
      
    } else { // End of block.
      // Set to execute the remaining steps and no phase correction upon finishing the block. 
      prep_segment->dist_per_tick = ceil( prep.steps_remaining*time_var ); // (mult*step/isr_tic)
      prep_segment->phase_dist = 0;
      prep_segment->n_step = ceil(prep.steps_remaining);
      
      prep.step_events_remaining -= prep_segment->n_step;
      if (prep.step_events_remaining > 0) {
        sys.state = STATE_QUEUED;
        pl_block->entry_speed_sqr = 0.0;
        prep.current_speed = 0.0;
        prep.steps_remaining = prep.step_events_remaining;
        pl_block->millimeters = prep.steps_remaining/prep.step_per_mm;
        prep.millimeters_remaining = pl_block->millimeters;
        pl_block = NULL;
        prep.flag_partial_block  = true;
        plan_cycle_reinitialize();
      } else {
        // The planner block is complete. All steps are set to be executed in the segment buffer.
        // TODO: Ignore this for feed holds. Need to recalculate the planner buffer at this time.
        pl_block = NULL;
        plan_discard_current_block();
      }

    }


    
    // New step segment initialization completed. Increment segment buffer indices.
    segment_buffer_head = segment_next_head;
    if ( ++segment_next_head == SEGMENT_BUFFER_SIZE ) { segment_next_head = 0; }

// int32_t blength = segment_buffer_head - segment_buffer_tail;
// if (blength < 0) { blength += SEGMENT_BUFFER_SIZE; } 
// printInteger(blength);
  } 
}      
