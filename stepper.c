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

#define RAMP_NOOP_CRUISE 0
#define RAMP_ACCEL 1
#define RAMP_DECEL 2

#define LOAD_NOOP 0
#define LOAD_SEGMENT 1
#define LOAD_BLOCK 2

#define SEGMENT_NOOP 0
#define SEGMENT_END_OF_BLOCK bit(0)
#define RAMP_CHANGE_ACCEL bit(1)
#define RAMP_CHANGE_DECEL bit(2)

#define MINIMUM_STEPS_PER_SEGMENT 1 // Don't change

#define SEGMENT_BUFFER_SIZE 6

// Stepper state variable. Contains running data and trapezoid variables.
typedef struct {
  // Used by the bresenham line algorithm
  int32_t counter_x,        // Counter variables for the bresenham line tracer
          counter_y, 
          counter_z;
  uint8_t segment_steps_remaining;  // Steps remaining in line segment motion

  // Used by inverse time algorithm to track step rate
  int32_t counter_dist;       // Inverse time distance traveled since last step event
  uint32_t ramp_rate;        // Inverse time distance traveled per interrupt tick
  uint32_t dist_per_tick;
  
  // Used by the stepper driver interrupt
  uint8_t execute_step;    // Flags step execution for each interrupt.
  uint8_t step_pulse_time; // Step pulse reset time after step rise
  uint8_t out_bits;        // The next stepping-bits to be output
  uint8_t load_flag;
  
  uint8_t counter_ramp;
  uint8_t ramp_type;
} stepper_t;
static stepper_t st;

// Stores stepper common data for executing steps in the segment buffer. Data can change mid-block when the
// planner updates the remaining block velocity profile with a more optimal plan or a feedrate override occurs.
// NOTE: Normally, this buffer is partially in-use, but, for the worst case scenario, it will never exceed
// the number of accessible stepper buffer segments (SEGMENT_BUFFER_SIZE-1).
typedef struct {  
  int32_t step_events_remaining; // Tracks step event count for the executing planner block
  uint32_t dist_per_step;               // Scaled distance to next step
  uint32_t initial_rate;         // Initialized step rate at re/start of a planner block
  uint32_t nominal_rate;         // The nominal step rate for this block in step_events/minute
  uint32_t rate_delta;           // The steps/minute to add or subtract when changing speed (must be positive)
  uint32_t current_approx_rate;  // Tracks the approximate segment rate to predict steps per segment to execute
  int32_t decelerate_after;      // Tracks when to initiate deceleration according to the planner block
  float mm_per_step;
} st_data_t;
static st_data_t segment_data[SEGMENT_BUFFER_SIZE-1];

// Primary stepper segment ring buffer. Contains small, short line segments for the stepper algorithm to execute,
// which are "checked-out" incrementally from the first block in the planner buffer. Once "checked-out", the steps
// in the segments buffer cannot be modified by the planner, where the remaining planner block steps still can.
typedef struct {
  uint8_t n_step;         // Number of step events to be executed for this segment
  uint8_t st_data_index;  // Stepper buffer common data index. Uses this information to execute this segment.
  uint8_t flag;           // Stepper algorithm bit-flag for special execution conditions.
} st_segment_t;
static st_segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

// Step segment ring buffer indices
static volatile uint8_t segment_buffer_tail;
static volatile uint8_t segment_buffer_head;
static uint8_t segment_next_head;

static volatile uint8_t busy;   // Used to avoid ISR nesting of the "Stepper Driver Interrupt". Should never occur though.
static plan_block_t *pl_current_block;  // A pointer to the planner block currently being traced
static st_segment_t *st_current_segment;
static st_data_t *st_current_data;

// Pointers for the step segment being prepped from the planner buffer. Accessed only by the
// main program. Pointers may be planning segments or planner blocks ahead of what being executed.
static plan_block_t *pl_prep_block;   // Pointer to the planner block being prepped
static st_data_t *st_prep_data;       // Pointer to the stepper common data being prepped
static uint8_t pl_prep_index;         // Index of planner block being prepped
static uint8_t st_data_prep_index;    // Index of stepper common data block being prepped
static uint8_t pl_partial_block_flag; // Flag indicating the planner has modified the prepped planner block


/*        __________________________
         /|                        |\     _________________         ^
        / |                        | \   /|               |\        |
       /  |                        |  \ / |               | \       s
      /   |                        |   |  |               |  \      p
     /    |                        |   |  |               |   \     e
    +-----+------------------------+---+--+---------------+----+    e
    |               BLOCK 1            |      BLOCK 2          |    d

                            time ----->
  
   The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates by block->rate_delta
   until reaching cruising speed block->nominal_rate, and/or until step_events_remaining reaches block->decelerate_after
   after which it decelerates until the block is completed. The driver uses constant acceleration, which is applied as
   +/- block->rate_delta velocity increments by the midpoint rule at each ACCELERATION_TICKS_PER_SECOND.
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
    st.execute_step = false;
    st.load_flag = LOAD_BLOCK;
    
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
   - Measure time in ISR. Typical and worst-case. Should be virtually identical to last algorithm.
     There are no major changes to the base operations of this ISR with the new segment buffer.
   - Write how the acceleration counters work and why they are set at half via mid-point rule.
   - Determine if placing the position counters elsewhere (or change them to 8-bit variables that
     are added to the system position counters at the end of a segment) frees up cycles.
   - Write a blurb about how the acceleration should be handled within the ISR. All of the 
     time/step/ramp counters accurately keep track of the remainders and phasing of the variables
     with time. This means we do not have to compute them via expensive floating point beforehand.
   - Need to do an analysis to determine if these counters are really that much cheaper. At least
     find out when it isn't anymore. Particularly when the ISR is at a very high frequency.
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
  if (st.load_flag != LOAD_NOOP) {
    
    // Anything in the buffer? If so, load and initialize next step segment.
    if (segment_buffer_head != segment_buffer_tail) {
      
      // Initialize new step segment and load number of steps to execute
      st_current_segment = &segment_buffer[segment_buffer_tail];
      st.segment_steps_remaining = st_current_segment->n_step;
    
      // If the new segment starts a new planner block, initialize stepper variables and counters.
      // NOTE: For new segments only, the step counters are not updated to ensure step phasing is continuous.
      if (st.load_flag == LOAD_BLOCK) {
        pl_current_block = plan_get_current_block(); // Should always be there. Stepper buffer handles this.
        st_current_data = &segment_data[segment_buffer[segment_buffer_tail].st_data_index];
        
        // Initialize direction bits for block. Set execute flag to set directions bits upon next ISR tick.
        st.out_bits = pl_current_block->direction_bits ^ settings.invert_mask;
        st.execute_step = true;
        
        // Initialize Bresenham line counters
        st.counter_x = (pl_current_block->step_event_count >> 1);
        st.counter_y = st.counter_x;
        st.counter_z = st.counter_x;
        
        // Initialize inverse time, step rate data, and acceleration ramp counters
        st.counter_dist = st_current_data->dist_per_step;  // dist_per_step always greater than ramp_rate.   
        st.ramp_rate = st_current_data->initial_rate; 
        st.counter_ramp = ISR_TICKS_PER_ACCELERATION_TICK/2;  // Initialize ramp counter via midpoint rule
        st.ramp_type = RAMP_NOOP_CRUISE; // Initialize as no ramp operation. Corrected later if necessary. 

        // Ensure the initial step rate exceeds the MINIMUM_STEP_RATE.
        if (st.ramp_rate < MINIMUM_STEP_RATE) { st.dist_per_tick = MINIMUM_STEP_RATE; }
        else { st.dist_per_tick = st.ramp_rate; }
      }

      // Check if ramp conditions have changed. If so, update ramp counters and control variables.
      if ( st_current_segment->flag & (RAMP_CHANGE_DECEL | RAMP_CHANGE_ACCEL) ) {
        /* Compute correct ramp count for a ramp change. Upon a switch from acceleration to deceleration,
           or vice-versa, the new ramp count must be set to trigger the next acceleration tick equal to
           the number of ramp ISR ticks counted since the last acceleration tick. This is ensures the 
           ramp is executed exactly as the plan dictates. Otherwise, when a ramp begins from a known
           rate (nominal/cruise or initial), the ramp count must be set to ISR_TICKS_PER_ACCELERATION_TICK/2
           as mandated by the mid-point rule. For the latter conditions, the ramp count have been
           initialized such that the following computation is still correct. */
        st.counter_ramp = ISR_TICKS_PER_ACCELERATION_TICK-st.counter_ramp;
        if ( st_current_segment->flag & RAMP_CHANGE_DECEL ) { st.ramp_type = RAMP_DECEL; }
        else { st.ramp_type = RAMP_ACCEL; }
      }
      
      st.load_flag = LOAD_NOOP; // Segment motion loaded. Set no-operation flag to skip during execution.

    } else {
      // Can't discard planner block here if a feed hold stops in middle of block.
      st_go_idle();
      bit_true(sys.execute,EXEC_CYCLE_STOP); // Flag main program for cycle end
      return; // Nothing to do but exit.
    }  
    
  } 
  
  // Adjust inverse time counter for ac/de-celerations
  if (st.ramp_type) { // Ignored when ramp type is RAMP_NOOP_CRUISE
    st.counter_ramp--; // Tick acceleration ramp counter
    if (st.counter_ramp == 0) { // Adjust step rate when its time
      st.counter_ramp = ISR_TICKS_PER_ACCELERATION_TICK; // Reload ramp counter
      if (st.ramp_type == RAMP_ACCEL) { // Adjust velocity for acceleration
        st.ramp_rate += st_current_data->rate_delta;
        if (st.ramp_rate >= st_current_data->nominal_rate) { // Reached nominal rate.
          st.ramp_rate = st_current_data->nominal_rate; // Set cruising velocity
          st.ramp_type = RAMP_NOOP_CRUISE; // Set ramp flag to cruising
          st.counter_ramp = ISR_TICKS_PER_ACCELERATION_TICK/2; // Re-initialize counter for next ramp change.
        }
      } else { // Adjust velocity for deceleration.
        if (st.ramp_rate > st_current_data->rate_delta) {
          st.ramp_rate -= st_current_data->rate_delta;
        } else { // Moving near zero feed rate. Gracefully slow down.
          st.ramp_rate >>= 1; // Integer divide by 2 until complete. Also prevents overflow.
        }
      }
      // Adjust for minimum step rate, but retain operating ramp rate for accurate velocity tracing.
      if (st.ramp_rate < MINIMUM_STEP_RATE) { st.dist_per_tick = MINIMUM_STEP_RATE; }
      else { st.dist_per_tick = st.ramp_rate; }
    }
  }
  
  // Iterate inverse time counter. Triggers each Bresenham step event.
  st.counter_dist -= st.dist_per_tick; 
  
  // Execute Bresenham step event, when it's time to do so.
  if (st.counter_dist < 0) {  
    st.counter_dist += st_current_data->dist_per_step; // Reload inverse time counter

    st.out_bits = pl_current_block->direction_bits; // Reset out_bits and reload direction bits
    st.execute_step = true;
    
    // Execute step displacement profile by Bresenham line algorithm
    st.counter_x -= pl_current_block->steps[X_AXIS];
    if (st.counter_x < 0) {
      st.out_bits |= (1<<X_STEP_BIT);
      st.counter_x += pl_current_block->step_event_count;
      if (st.out_bits & (1<<X_DIRECTION_BIT)) { sys.position[X_AXIS]--; }
      else { sys.position[X_AXIS]++; }
    }
    st.counter_y -= pl_current_block->steps[Y_AXIS];
    if (st.counter_y < 0) {
      st.out_bits |= (1<<Y_STEP_BIT);
      st.counter_y += pl_current_block->step_event_count;
      if (st.out_bits & (1<<Y_DIRECTION_BIT)) { sys.position[Y_AXIS]--; }
      else { sys.position[Y_AXIS]++; }
    }
    st.counter_z -= pl_current_block->steps[Z_AXIS];
    if (st.counter_z < 0) {
      st.out_bits |= (1<<Z_STEP_BIT);
      st.counter_z += pl_current_block->step_event_count;
      if (st.out_bits & (1<<Z_DIRECTION_BIT)) { sys.position[Z_AXIS]--; }
      else { sys.position[Z_AXIS]++; }
    }

    // Check step events for trapezoid change or end of block.
    st.segment_steps_remaining--; // Decrement step events count    
    if (st.segment_steps_remaining == 0) {
      // Line move is complete, set load line flag to check for new move.
      // Check if last line move in planner block. Discard if so.
      if (st_current_segment->flag & SEGMENT_END_OF_BLOCK) {
        plan_discard_current_block();
        st.load_flag = LOAD_BLOCK;
      } else {
        st.load_flag = LOAD_SEGMENT;
      }
      
      // Discard current segment by advancing buffer tail index
      if ( ++segment_buffer_tail == SEGMENT_BUFFER_SIZE) { segment_buffer_tail = 0; }
    }

    st.out_bits ^= settings.invert_mask;  // Apply step port invert mask    
  }
  busy = false;
// SPINDLE_ENABLE_PORT ^= 1<<SPINDLE_ENABLE_BIT;  
}


// The Stepper Port Reset Interrupt: Timer0 OVF interrupt handles the falling edge of the step
// pulse. This should always trigger before the next Timer2 COMPA interrupt and independently
// finish, if Timer2 is disabled after completing a move.
ISR(TIMER0_OVF_vect)
{
  STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | (settings.invert_mask & STEP_MASK); 
  TCCR0B = 0; // Disable timer until needed.
}


// Reset and clear stepper subsystem variables
void st_reset()
{
  memset(&st, 0, sizeof(st));
  
  st.load_flag = LOAD_BLOCK;
  busy = false;

  pl_current_block = NULL; // Planner block pointer used by stepper algorithm
  pl_prep_block = NULL;  // Planner block pointer used by segment buffer
  pl_prep_index =   0; // Planner buffer indices are also reset to zero.
  st_data_prep_index = 0;
  
  segment_buffer_tail = 0;
  segment_buffer_head = 0; // empty = tail
  segment_next_head = 1;
    
  pl_partial_block_flag = false;
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
//   if (pl_current_block != NULL) {
    // Replan buffer from the feed hold stop location.
    
    // TODO: Need to add up all of the step events in the current planner block to give 
    // back to the planner. Should only need it for the current block. 
    // BUT! The planner block millimeters is all changed and may be changed into the next
    // planner block. The block millimeters would need to be recalculated via step counts
    // and the mm/step variable.
    // OR. Do we plan the feed hold itself down with the planner.
    
//     plan_cycle_reinitialize(st_current_data->step_events_remaining);
//     st.ramp_type = RAMP_ACCEL;
//     st.counter_ramp = ISR_TICKS_PER_ACCELERATION_TICK/2; 
//     st.ramp_rate = 0;
//     sys.state = STATE_QUEUED;
//   } else {
//     sys.state = STATE_IDLE;
//   }
    sys.state = STATE_IDLE;

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
   Currently, the segment buffer conservatively holds roughly up to 40-60 msec of steps. 

   NOTE: The segment buffer executes a set number of steps over an approximate time period.
   If we try to execute over a fixed time period, it is difficult to guarantee or predict 
   how many steps will execute over it, especially when the step pulse phasing between the
   neighboring segments must also be kept consistent. Meaning that, if the last segment step 
   pulses right before a segment end, the next segment must delay its first pulse so that the
   step pulses are consistently spaced apart over time to keep the step pulse train nice and
   smooth. Keeping track of phasing and ensuring that the exact number of steps are executed
   as defined by the planner block, the related computational overhead can get quickly and
   prohibitively expensive, especially in real-time.
     Since the stepper algorithm automatically takes care of the step pulse phasing with
   its ramp and inverse time counters by retaining the count remainders, we don't have to
   explicitly and expensively track and synchronize the exact number of steps, time, and
   phasing of steps. All we need to do is approximate the number of steps in each segment 
   such that the segment buffer has enough execution time for the main program to do what 
   it needs to do and refill it when it comes back. In other words, we just need to compute
   a cheap approximation of the current velocity and the number of steps over it. 
*/

/* 
   TODO: Figure out how to enforce a deceleration when a feedrate override is reduced. 
     The problem is that when an override is reduced, the planner may not plan back to 
     the current rate. Meaning that the velocity profiles for certain conditions no longer
     are trapezoidal or triangular. For example, if the current block is cruising at a
     nominal rate and the feedrate override is reduced, the new nominal rate will now be
     lower. The velocity profile must first decelerate to the new nominal rate and then 
     follow on the new plan. So the remaining velocity profile will have a decelerate, 
     cruise, and another decelerate.
        Another issue is whether or not a feedrate override reduction causes a deceleration
     that acts over several planner blocks. For example, say that the plan is already 
     heavily decelerating throughout it, reducing the feedrate will not do much to it. So,
     how do we determine when to resume the new plan? How many blocks do we have to wait 
     until the new plan intersects with the deceleration curve? One plus though, the 
     deceleration will never be more than the number of blocks in the entire planner buffer,
     but it theoretically can be equal to it when all planner blocks are decelerating already.
*/
void st_prep_buffer()
{
  if (sys.state == STATE_QUEUED) { return; } // Block until a motion state is issued
  while (segment_buffer_tail != segment_next_head) { // Check if we need to fill the buffer.
    
    // Initialize new segment
    st_segment_t *prep_segment = &segment_buffer[segment_buffer_head];
    prep_segment->flag = SEGMENT_NOOP;
    
    // Determine if we need to load a new planner block.
    if (pl_prep_block == NULL) {
      pl_prep_block = plan_get_block_by_index(pl_prep_index); // Query planner for a queued block
      if (pl_prep_block == NULL) { return; } // No planner blocks. Exit.
        
      // Increment stepper common data index 
      if ( ++st_data_prep_index == (SEGMENT_BUFFER_SIZE-1) ) { st_data_prep_index = 0; }
        
      // Check if the planner has re-computed this block mid-execution. If so, push the previous segment
      // data. Otherwise, prepare a new segment data for the new planner block.
      if (pl_partial_block_flag) {
        
        // Prepare new shared segment block data and copy the relevant last segment block data.
        st_data_t *last_st_prep_data;
        last_st_prep_data = st_prep_data;
        st_prep_data = &segment_data[st_data_prep_index];
            
        st_prep_data->step_events_remaining = last_st_prep_data->step_events_remaining;
        st_prep_data->rate_delta = last_st_prep_data->rate_delta;
        st_prep_data->dist_per_step = last_st_prep_data->dist_per_step;
        st_prep_data->nominal_rate = last_st_prep_data->nominal_rate; // TODO: Feedrate overrides recomputes this.
       
        st_prep_data->mm_per_step = last_st_prep_data->mm_per_step;

        pl_partial_block_flag = false; // Reset flag
        
      } else {
      
        // Prepare commonly shared planner block data for the ensuing segment buffer moves ad-hoc, since 
        // the planner buffer can dynamically change the velocity profile data as blocks are added.
        st_prep_data = &segment_data[st_data_prep_index];
      
        // Initialize Bresenham variables
        st_prep_data->step_events_remaining = pl_prep_block->step_event_count;

        // Convert planner block velocity profile data to stepper rate and step distance data.
        st_prep_data->nominal_rate = ceil(sqrt(pl_prep_block->nominal_speed_sqr)*(INV_TIME_MULTIPLIER/(60.0*ISR_TICKS_PER_SECOND))); // (mult*mm/isr_tic)
        st_prep_data->rate_delta = ceil(pl_prep_block->acceleration*
          ((INV_TIME_MULTIPLIER/(60.0*60.0))/(ISR_TICKS_PER_SECOND*ACCELERATION_TICKS_PER_SECOND))); // (mult*mm/isr_tic/accel_tic)
        st_prep_data->dist_per_step = ceil((pl_prep_block->millimeters*INV_TIME_MULTIPLIER)/pl_prep_block->step_event_count); // (mult*mm/step)
        
        // TODO: Check if we really need to store this.
        st_prep_data->mm_per_step = pl_prep_block->millimeters/pl_prep_block->step_event_count;        
    
      }
      
      // Convert planner entry speed to stepper initial rate. 
      st_prep_data->initial_rate = ceil(sqrt(pl_prep_block->entry_speed_sqr)*(INV_TIME_MULTIPLIER/(60.0*ISR_TICKS_PER_SECOND))); // (mult*mm/isr_tic)

      // TODO: Nominal rate changes with feedrate override.
      // st_prep_data->nominal_rate = ceil(sqrt(pl_prep_block->nominal_speed_sqr)*(INV_TIME_MULTIPLIER/(60.0*ISR_TICKS_PER_SECOND))); // (mult*mm/isr_tic)

      st_prep_data->current_approx_rate = st_prep_data->initial_rate;

      // Calculate the planner block velocity profile type, determine deceleration point, and initial ramp.
      float mm_decelerate_after = plan_calculate_velocity_profile(pl_prep_index);
      st_prep_data->decelerate_after = ceil( mm_decelerate_after/st_prep_data->mm_per_step );
      if (st_prep_data->decelerate_after > 0) { // If 0, RAMP_CHANGE_DECEL flag is set later.
        if (st_prep_data->initial_rate != st_prep_data->nominal_rate) { prep_segment->flag = RAMP_CHANGE_ACCEL; }
      }
    }

    // Set new segment to point to the current segment data block.
    prep_segment->st_data_index = st_data_prep_index;

    // Approximate the velocity over the new segment using the already computed rate values.
    // NOTE: This assumes that each segment will have an execution time roughly equal to every ACCELERATION_TICK.
    // We do this to minimize memory and computational requirements. However, this could easily be replaced with
    // a more exact approximation or have an user-defined time per segment, if CPU and memory overhead allows.
    if (st_prep_data->decelerate_after <= 0) {
      if (st_prep_data->decelerate_after == 0) { prep_segment->flag = RAMP_CHANGE_DECEL; } // Set segment deceleration flag
      else { st_prep_data->current_approx_rate -= st_prep_data->rate_delta; }
      if (st_prep_data->current_approx_rate < st_prep_data->rate_delta) { st_prep_data->current_approx_rate >>= 1; }
    } else {    
      if (st_prep_data->current_approx_rate < st_prep_data->nominal_rate) {
        st_prep_data->current_approx_rate += st_prep_data->rate_delta;
        if (st_prep_data->current_approx_rate > st_prep_data->nominal_rate) {
          st_prep_data->current_approx_rate = st_prep_data->nominal_rate;
        }
      }
    }
    
    // TODO: Look into replacing the following dist_per_step divide with multiplying its inverse to save cycles.
    
    // Compute the number of steps in the prepped segment based on the approximate current rate.
    // NOTE: The dist_per_step divide cancels out the INV_TIME_MULTIPLIER and converts the rate value to steps.
    prep_segment->n_step = ceil(max(MINIMUM_STEP_RATE,st_prep_data->current_approx_rate)*
                                (ISR_TICKS_PER_SECOND/ACCELERATION_TICKS_PER_SECOND)/st_prep_data->dist_per_step);    
    // NOTE: Ensures it moves for very slow motions, but MINIMUM_STEP_RATE should always set this too. Perhaps
    // a compile-time check to see if MINIMUM_STEP_RATE is set high enough is all that is needed.
    prep_segment->n_step = max(prep_segment->n_step,MINIMUM_STEPS_PER_SEGMENT);
    // NOTE: As long as the ACCELERATION_TICKS_PER_SECOND is valid, n_step should never exceed 255 and overflow.
    // prep_segment->n_step = min(prep_segment->n_step,MAXIMUM_STEPS_PER_BLOCK); // Prevent unsigned int8 overflow.
    
    // Check if n_step exceeds steps remaining in planner block. If so, truncate.
    if (prep_segment->n_step > st_prep_data->step_events_remaining) { 
      prep_segment->n_step = st_prep_data->step_events_remaining; 
    }

    // Check if n_step crosses decelerate point in block. If so, truncate to ensure the deceleration
    // ramp counters are set correctly during execution.
    if (st_prep_data->decelerate_after > 0) {
      if (prep_segment->n_step > st_prep_data->decelerate_after) { 
        prep_segment->n_step = st_prep_data->decelerate_after; 
      }
    }
 
    // Update stepper common data variables.
    st_prep_data->decelerate_after -= prep_segment->n_step;
    st_prep_data->step_events_remaining -= prep_segment->n_step;
    
    // Check for end of planner block
    if ( st_prep_data->step_events_remaining == 0 ) { 
    
      // TODO: When a feed hold ends, the step_events_remaining will also be zero, even though a block
      // have partially been completed. We need to flag the stepper algorithm to indicate a stepper shutdown
      // when complete, but not remove the planner block unless it truly is the end of the block (rare).
    
      // Set EOB bitflag so stepper algorithm discards the planner block after this segment completes.
      prep_segment->flag |= SEGMENT_END_OF_BLOCK;
      // Move planner pointer to next block and flag to load a new block for the next segment.
      pl_prep_index = plan_next_block_index(pl_prep_index);
      pl_prep_block = NULL;
    }    

    // New step segment completed. Increment segment buffer indices.
    segment_buffer_head = segment_next_head;
    if ( ++segment_next_head == SEGMENT_BUFFER_SIZE ) { segment_next_head = 0; }
    
// long a = prep_segment->n_step;    
// printInteger(a);
// printString(" ");

  } 
}      

uint8_t st_get_prep_block_index() 
{
// Returns only the index but doesn't state if the block has been partially executed. How do we simply check for this?
  return(pl_prep_index);
}

void st_fetch_partial_block_parameters(uint8_t block_index, float *millimeters_remaining, uint8_t *is_decelerating)
{
 // if called, can we assume that this always changes and needs to be updated? if so, then
 // we can perform all of the segment buffer setup tasks here to make sure the next time
 // the segments are loaded, the st_data buffer is updated correctly.
 // !!! Make sure that this is always pointing to the correct st_prep_data block. 
 
 // When a mid-block acceleration occurs, we have to make sure the ramp counters are updated
 // correctly, much in the same fashion as the deceleration counters. Need to think about this 
 // make sure this is right, but i'm pretty sure it is.
  
  // TODO: NULL means that the segment buffer has just completed a planner block. Clean up!
  if (pl_prep_block != NULL) {
    *millimeters_remaining = st_prep_data->step_events_remaining*st_prep_data->mm_per_step;
    if (st_prep_data->decelerate_after > 0) { *is_decelerating = false; } 
    else { *is_decelerating = true; }

    // Flag for new prep_block when st_prep_buffer() is called after the planner recomputes.
    pl_partial_block_flag = true;
    pl_prep_block = NULL;
  }
  return;
}
