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

#define RAMP_ACCEL 0
#define RAMP_CRUISE 1
#define RAMP_DECEL 2

#define LOAD_NOOP 0
#define LOAD_SEGMENT 1
#define LOAD_BLOCK 2

#define SEGMENT_NOOP 0
#define SEGMENT_END_OF_BLOCK bit(0)
#define RAMP_CHANGE_ACCEL bit(1)
#define RAMP_CHANGE_DECEL bit(2)

#define SEGMENT_BUFFER_SIZE 6

#define DT_SEGMENT (1.0/(ACCELERATION_TICKS_PER_SECOND*60.0))

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
  uint8_t st_block_index;  // Stepper block data index. Uses this information to execute this segment.
  int32_t phase_dist;
  int32_t dist_per_tick;
} segment_t;
static segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

// Stepper state variable. Contains running data and trapezoid variables.
typedef struct {
  // Used by the bresenham line algorithm
  int32_t counter_x,        // Counter variables for the bresenham line tracer
          counter_y, 
          counter_z;

  // Used by inverse time algorithm to track step rate
  int32_t counter_dist;    // Inverse time distance traveled since last step event
  
  // Used by the stepper driver interrupt
  uint8_t execute_step;     // Flags step execution for each interrupt.
  uint8_t step_pulse_time;  // Step pulse reset time after step rise
  uint8_t out_bits;         // The next stepping-bits to be output

  uint8_t step_count;       // Steps remaining in line segment motion  
  uint8_t exec_block_index; // Tracks the current st_block index. Change indicates new block.
  st_block_t *exec_block;  // Pointer to the block data for the segment being executed
  segment_t *exec_segment; // Pointer to the segment being executed
} stepper_t;
static stepper_t st;

// Step segment ring buffer indices
static volatile uint8_t segment_buffer_tail;
static volatile uint8_t segment_buffer_head;
static uint8_t segment_next_head;

static volatile uint8_t busy;   // Used to avoid ISR nesting of the "Stepper Driver Interrupt". Should never occur though.

// Pointers for the step segment being prepped from the planner buffer. Accessed only by the
// main program. Pointers may be planning segments or planner blocks ahead of what being executed.
static plan_block_t *pl_block;     // Pointer to the planner block being prepped
static st_block_t *st_prep_block;  // Pointer to the stepper block data being prepped 

typedef struct {
  uint8_t st_block_index;      // Index of stepper common data block being prepped
  uint8_t partial_block_flag;  // Flag indicating the planner has modified the prepped planner block

  float step_per_mm;
  float step_events_remaining; // Tracks step event count for the executing planner block
//   int32_t step_events_remaining;
  float step_remainder;
  
  uint8_t ramp_type;
  float current_speed;
  float maximum_speed;
  float exit_speed;
  float accelerate_until;
  float decelerate_after;  
} st_prep_t;
static st_prep_t prep;


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
        
        // Initialize Bresenham line counters
        st.counter_x = (st.exec_block->step_event_count >> 1);
        st.counter_y = st.counter_x;
        st.counter_z = st.counter_x;
        
        // Initialize inverse time, step rate data, and acceleration ramp counters
        st.counter_dist = INV_TIME_MULTIPLIER;  // dist_per_step always greater than dist_per_tick.
      }
      
    } else {
      // Segment buffer empty. Shutdown.
      st_go_idle();
      bit_true(sys.execute,EXEC_CYCLE_STOP); // Flag main program for cycle end
      return; // Nothing to do but exit.
    }  
    
  } 
    
  // Iterate inverse time counter. Triggers each Bresenham step event.
  st.counter_dist -= st.exec_segment->dist_per_tick; 
  
  // Execute Bresenham step event, when it's time to do so.
  if (st.counter_dist < 0) {
    if (st.step_count != 0) {  // Block phase correction from executing step. 
      st.counter_dist += INV_TIME_MULTIPLIER; // Reload inverse time counter
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
    if (st.exec_segment->phase_dist > st.counter_dist) { 
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
//     plan_cycle_reinitialize(st_exec_block->step_events_remaining);
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

    
    // -----------------------------------------------------------------------------------
    // Determine if we need to load a new planner block. If so, prepare step data.
    if (pl_block == NULL) {
      pl_block = plan_get_current_block(); // Query planner for a queued block
      if (pl_block == NULL) { return; } // No planner blocks. Exit.
      
// SPINDLE_ENABLE_PORT ^= 1<<SPINDLE_ENABLE_BIT; 
                
      // Check if the planner has re-computed this block mid-execution.
      if (prep.partial_block_flag) {
        // Retain last Bresenham data, but recompute the velocity profile.      
        prep.partial_block_flag = false; // Reset flag
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

        // Initialize planner block step count, unit distance data, and remainder tracker.
        prep.step_per_mm = ((float)st_prep_block->step_event_count)/pl_block->millimeters;        
        prep.step_events_remaining = st_prep_block->step_event_count;
        prep.step_remainder = 0.0;   
      }
      
      // Compute the prepped planner block velocity profile to be traced by stepper algorithm. 
      prep.current_speed = sqrt(pl_block->entry_speed_sqr);
      prep.exit_speed = plan_get_exec_block_exit_speed(); 

      // Determine velocity profile based on the 7 possible types: Cruise-only, cruise-deceleration,
      // acceleration-cruise, acceleration-only, deceleration-only, full-trapezoid, and triangle.
      prep.ramp_type = RAMP_ACCEL;
      float exit_speed_sqr = prep.exit_speed*prep.exit_speed;
      float inv_2_accel = 0.5/pl_block->acceleration;
      float intersection_dist =
              0.5*(pl_block->millimeters+inv_2_accel*(pl_block->entry_speed_sqr-exit_speed_sqr));
      if (intersection_dist > 0.0) {
        if (intersection_dist < pl_block->millimeters) { // Either trapezoid or triangle types
          // NOTE: For acceleration-cruise trapezoid, following calculation will be 0.0.
          prep.decelerate_after = inv_2_accel*(pl_block->nominal_speed_sqr-exit_speed_sqr);
          if (prep.decelerate_after < intersection_dist) { // Trapezoid type
            prep.maximum_speed = sqrt(pl_block->nominal_speed_sqr);
            if (pl_block->entry_speed_sqr == pl_block->nominal_speed_sqr) { 
              // Cruise-deceleration or cruise-only type.
              prep.ramp_type = RAMP_CRUISE;
              prep.accelerate_until = pl_block->millimeters; 
            } else {
              // Full-trapezoid or acceleration-cruise types
              prep.accelerate_until = 
                pl_block->millimeters-inv_2_accel*(pl_block->nominal_speed_sqr-pl_block->entry_speed_sqr); 
            }
          } else { // Triangle type
            prep.accelerate_until = intersection_dist;
            prep.decelerate_after = intersection_dist;
            prep.maximum_speed = sqrt(2.0*pl_block->acceleration*intersection_dist+exit_speed_sqr);
          }          
        } else { // Deceleration-only type
          prep.ramp_type = RAMP_DECEL;
          prep.maximum_speed = prep.current_speed;
          prep.accelerate_until = pl_block->millimeters;
          prep.decelerate_after = pl_block->millimeters;
        }
      } else { // Acceleration-only type
        prep.maximum_speed = prep.exit_speed;
        prep.accelerate_until = 0.0;
        prep.decelerate_after = 0.0;
      }
        
    }

    // Initialize new segment
    segment_t *prep_segment = &segment_buffer[segment_buffer_head];

    // Set new segment to point to the current segment data block.
    prep_segment->st_block_index = prep.st_block_index;

    /* -----------------------------------------------------------------------------------
       Compute the average velocity of this new segment by determining the total distance
       traveled over the segment time DT_SEGMENT. This section attempts to create a full
       segment based on the current ramp conditions. If the segment is incomplete and 
       terminates upon a ramp change, the next section will attempt to fill the remaining 
       segment execution time. However, if an incomplete segment terminates at the end of
       the planner block, the segment execution time is less than DT_SEGMENT and the new
       segment will execute over this truncated execution time.
    */   
    float dt = 0.0;
    float mm_remaining = pl_block->millimeters;
    float dt_var = DT_SEGMENT;
    float mm_var;
    do {
      switch (prep.ramp_type) {
        case RAMP_ACCEL: 
          // NOTE: Acceleration ramp always computes during first loop only.
          mm_remaining -= DT_SEGMENT*(prep.current_speed + pl_block->acceleration*(0.5*DT_SEGMENT));
          if (mm_remaining < prep.accelerate_until) { // End of acceleration ramp.
            // Acceleration-cruise, acceleration-deceleration ramp junction, or end of block.
            mm_remaining = prep.accelerate_until; // NOTE: 0.0 at EOB
            dt_var = 2.0*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            if (mm_remaining == prep.decelerate_after) { prep.ramp_type = RAMP_DECEL; }
            else { prep.ramp_type = RAMP_CRUISE; }
            prep.current_speed = prep.maximum_speed;
          } else { // Acceleration only. 
            prep.current_speed += pl_block->acceleration*dt_var;
          }
          break;
        case RAMP_CRUISE: 
          // NOTE: mm_var used to retain the last mm_remaining for incomplete segment dt_var calculations.
          mm_var = mm_remaining - prep.maximum_speed*dt_var;
          if (mm_var < prep.decelerate_after) { // End of cruise. 
            // Cruise-deceleration junction or end of block.
            dt_var = (mm_remaining - prep.decelerate_after)/prep.maximum_speed;
            mm_remaining = prep.decelerate_after; // NOTE: 0.0 at EOB
            prep.ramp_type = RAMP_DECEL;
          } else { // Cruising only.         
            mm_remaining = mm_var; 
          } 
          break;
        default: // case RAMP_DECEL:
          // NOTE: mm_var used to catch negative decelerate distance values near zero speed.
          mm_var = dt_var*(prep.current_speed - 0.5*pl_block->acceleration*dt_var);
          if ((mm_var > 0.0) && (mm_var < pl_block->millimeters)) { // Deceleration only.
            prep.current_speed -= pl_block->acceleration*dt_var;
            // Check for near-zero speed and prevent divide by zero in rare scenarios.
            if (prep.current_speed <= prep.exit_speed) { mm_remaining = 0.0; }
            else { mm_remaining -= mm_var; }
          } else { // End of block.
            dt_var = 2.0*mm_remaining/(prep.current_speed+prep.exit_speed);
            mm_remaining = 0.0;
            // prep.current_speed = prep.exit_speed;
          }
      }
      dt += dt_var;
      if (dt < DT_SEGMENT) { dt_var = DT_SEGMENT - dt; } // **Incomplete** At ramp junction.
      else { break; } // **Complete** Exit loop. Segment execution time maxed.      
    } while ( mm_remaining > 0.0 ); // **Complete** Exit loop. End of planner block.
        
    /*
    float mm_remaining;
    float dt = DT_SEGMENT;    
    if (pl_block->millimeters > prep.accelerate_until) { // [Acceleration Ramp]
      mm_remaining = pl_block->millimeters - DT_SEGMENT*(prep.current_speed + pl_block->acceleration*(0.5*DT_SEGMENT));
      if (mm_remaining < prep.accelerate_until) { // **Incomplete** Acceleration ramp end.
        // Acceleration-cruise, acceleration-deceleration ramp junction, or end of block.
        mm_remaining = prep.accelerate_until; // NOTE: 0.0 at EOB
        dt = 2.0*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
        prep.current_speed = prep.maximum_speed;
      } else { // **Complete** Acceleration only. 
        prep.current_speed += pl_block->acceleration*DT_SEGMENT;
        prep.current_speed = min(prep.maximum_speed,prep.current_speed);
      }
    } else if (pl_block->millimeters > prep.decelerate_after) { // [No Ramp. Cruising]
        mm_remaining = pl_block->millimeters - prep.maximum_speed*DT_SEGMENT;
        if (mm_remaining < prep.decelerate_after) { // **Incomplete** End of cruise. 
          // Cruise-deceleration junction or end of block.
          mm_remaining = prep.decelerate_after; // NOTE: 0.0 at EOB
          dt = (pl_block->millimeters-mm_remaining)/prep.maximum_speed;
        } // Otherwise **Complete** Cruising only. 
    } else {  // [Deceleration Ramp]
      mm_remaining = DT_SEGMENT*(prep.current_speed - 0.5*pl_block->acceleration*DT_SEGMENT);
      if ((mm_remaining > 0.0) && (mm_remaining < pl_block->millimeters)) { // **Complete** Deceleration only.
        prep.current_speed -= pl_block->acceleration*DT_SEGMENT;
        if (prep.current_speed <= prep.exit_speed) { // Round off error fix. Prevents divide by zero.
          mm_remaining = 0.0;
        } else { 
          mm_remaining = pl_block->millimeters - mm_remaining; 
        }
      } else { // **Complete** End of block.
        mm_remaining = 0.0;
        dt = 2.0*pl_block->millimeters/(prep.current_speed+prep.exit_speed);
        // prep.current_speed = prep.exit_speed;
      }
    }
    
    
    /* -----------------------------------------------------------------------------------
       If segment is incomplete, attempt to fill the remaining segment execution time.
       NOTE: Segment remainder always spans a cruise and/or a deceleration ramp.        
    
    float partial_mm, dt_remainder;
    if ((dt < DT_SEGMENT) && (mm_remaining > 0.0)) {      
      dt_remainder = DT_SEGMENT-dt;
      
      // Attempt to fill incomplete segment with cruising profile. 
      if (mm_remaining > prep.decelerate_after) { // Cruising profile
        partial_mm = mm_remaining - prep.current_speed*dt_remainder;
        if (partial_mm < prep.decelerate_after) { // **Incomplete**
          dt += (mm_remaining-prep.decelerate_after)/prep.maximum_speed;
          mm_remaining = prep.decelerate_after;
          // current_speed = maximum_speed;
        } else { // **Complete** Segment filled. 
          mm_remaining = partial_mm; 
          dt = DT_SEGMENT;
        }
      }
      
      // Attempt to fill incomplete segment with deceleration ramp.
      if ((dt < DT_SEGMENT) && (mm_remaining > 0.0)) {  
      if (mm_remaining <= prep.decelerate_after) { // Deceleration ramp
        dt_remainder = DT_SEGMENT-dt;    
        partial_mm = dt_remainder*(prep.current_speed-0.5*pl_block->acceleration*dt_remainder);
        if ((partial_mm > 0.0) && (mm_remaining > partial_mm)) { // **Complete** Segment filled.
          prep.current_speed -= pl_block->acceleration*dt_remainder;
          if (prep.current_speed <= prep.exit_speed) {
            mm_remaining = 0.0;

          } else {
            mm_remaining -= partial_mm;
            dt = DT_SEGMENT;
          }          
        } else { // **Complete** End of block.
          dt += (2.0*mm_remaining/(prep.current_speed+prep.exit_speed));
          mm_remaining = 0.0;
          // prep.current_speed = prep.exit_speed;
        }
      }  
      }
    }
    */
//  printString(" Z");
//  printFloat(dt*(60.0*1000.0));
//  printString(" ");
//  printFloat(mm_remaining);
//  printString(" ");
//  printFloat(prep.current_speed);
//   printString("Z ");   
   
    /* -----------------------------------------------------------------------------------
       Compute segment step rate, steps to execute, and step phase correction parameters.
    */
//     float step_events;
//     if (mm_remaining > 0.0) { 
//       step_events = prep.step_per_mm*(pl_block->millimeters - mm_remaining); // Convert mm to steps
//       prep_segment->n_step = floor(step_events + prep.step_remainder);
//       if (prep_segment->n_step > prep.step_events_remaining) { // Prevent round-off overshoot
//         prep_segment->n_step = prep.step_events_remaining; 
//       }
//     } else { // Ensure all remaining steps are executed
//       step_events = prep.step_per_mm*pl_block->millimeters;
//       prep_segment->n_step = prep.step_events_remaining;
//     }
//     prep.step_events_remaining -= prep_segment->n_step;
//     
//     // Compute segment rate.
//     prep_segment->dist_per_tick =
//       ceil( (INV_TIME_MULTIPLIER/(60.0*ISR_TICKS_PER_SECOND)) * (step_events/dt) ); // (mult*step/isr_tic)
// 
//     if (prep.step_events_remaining > 0) {
//       // Compute step phase distance and update segment continuation parameters.
//       prep.step_remainder += step_events - prep_segment->n_step;
//       prep_segment->phase_dist = ceil(INV_TIME_MULTIPLIER-INV_TIME_MULTIPLIER*prep.step_remainder);
//       pl_block->millimeters = mm_remaining;
//       pl_block->entry_speed_sqr = prep.current_speed*prep.current_speed;
// 
//     } else { // End of block. Finish it out.
//       // The planner block is complete. All steps are set to be executed in the segment buffer.
//       // Move planner pointer to next block and flag to load a new block for the next segment.
//       prep_segment->phase_dist = INV_TIME_MULTIPLIER;   
//       pl_block = NULL;
//       plan_discard_current_block();
//     }

    if (mm_remaining > 0.0) { 

      float steps_remaining = prep.step_per_mm*mm_remaining; 
      prep_segment->dist_per_tick = ceil( (INV_TIME_MULTIPLIER/(60.0*ISR_TICKS_PER_SECOND))*
                                    ((prep.step_events_remaining-steps_remaining)/dt) ); // (mult*step/isr_tic)

      // Compute number of steps to execute and segment step phase correction. 
      prep_segment->n_step = ceil(prep.step_events_remaining)-ceil(steps_remaining);
      prep_segment->phase_dist = ceil(INV_TIME_MULTIPLIER*(1.0-ceil(steps_remaining)+steps_remaining));

      // Update step execution variables
      prep.step_events_remaining = steps_remaining;                                    
      pl_block->millimeters = mm_remaining;
      pl_block->entry_speed_sqr = prep.current_speed*prep.current_speed;

    } else { // End of block. Finish it out.

      prep_segment->dist_per_tick = ceil( (INV_TIME_MULTIPLIER/(60.0*ISR_TICKS_PER_SECOND))*
                                    prep.step_events_remaining/dt ); // (mult*step/isr_tic)
      prep_segment->phase_dist = INV_TIME_MULTIPLIER;

      // Set to execute the remaining steps and no phase correction upon finishing the block. 
      prep_segment->n_step = ceil(prep.step_events_remaining);

      
      // NOTE: Not required. Planner will ignore this block as it is now complete.
      // prep.step_events_remaining = 0.0;
      // pl_block->millimeters = 0.0;
      
      // The planner block is complete. All steps are set to be executed in the segment buffer.
      // Move planner pointer to next block and flag to load a new block for the next segment.
      pl_block = NULL;
      plan_discard_current_block();
    }
    
// long a = prep_segment->n_step;
//  printInteger(a);
//  printString(" ");
//  a = prep_segment->phase_dist;
//  printInteger(prep_segment->dist_per_tick);
//  printString(" ");
//  printFloat(prep.step_events_remaining);
//  printString(" "); 
//  printFloat(pl_block->millimeters);
//  printString(" ");


    // !!! PROBLEM. Step events remaining in floating point can limit the number of steps
    // we can accurately track, since floats have ~7.2 significant digits. However, this only
    // becomes a problem if there are more than 1,000,000, which translates to a CNC machine
    // with 200 step/mm and 5 meters of axis travel. Possible but unlikely. Could have more 
    // issues with user setting up their machine with too high of steps.
    
    // TODO: dist_per_tick must be less than INV_TIME_MULTIPLIER. A check can be made to 
    // make this a hard limit. Need to make sure this doesn't affect the velocity profiles..
    // it shouldn't. The same could said for the minimum allowable step rate too. This should
    // not affect the tracing of the profiles either.

   // Ensure the initial step rate exceeds the MINIMUM_STEP_RATE.
   // TODO: Use config.h error checking to do this. Otherwise, counters get screwy.

    // New step segment initialization completed. Increment segment buffer indices.
    segment_buffer_head = segment_next_head;
    if ( ++segment_next_head == SEGMENT_BUFFER_SIZE ) { segment_next_head = 0; }

int32_t blength = segment_buffer_head - segment_buffer_tail;
if (blength < 0) { blength += SEGMENT_BUFFER_SIZE; } 
printInteger(blength);
//     SPINDLE_ENABLE_PORT ^= 1<<SPINDLE_ENABLE_BIT;  
  } 
}      


// uint8_t st_get_prep_block_index() 
// {
// // Returns only the index but doesn't state if the block has been partially executed. How do we simply check for this?
//   return(prep.pl_block_index);
// }


void st_update_plan_block_parameters()
{ 
  if (pl_block != NULL) { // Ignore if at start of a new block.
    // Flag for new prep_block when st_prep_buffer() is called after the planner recomputes.
    prep.partial_block_flag = true;
    pl_block = NULL;
  }
}
