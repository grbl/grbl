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
#define LOAD_LINE 1
#define LOAD_BLOCK 2

#define ST_NOOP 0
#define ST_END_OF_BLOCK 1
#define ST_DECEL 2
#define ST_DECEL_EOB 3

#define SEGMENT_BUFFER_SIZE 6

// Stepper state variable. Contains running data and trapezoid variables.
typedef struct {
  // Used by the bresenham line algorithm
  int32_t counter_x,        // Counter variables for the bresenham line tracer
          counter_y, 
          counter_z;
  uint8_t segment_steps_remaining;  // Steps remaining in line segment motion

  // Used by inverse time algorithm to track step rate
  int32_t counter_d;       // Inverse time distance traveled since last step event
  uint32_t delta_d;        // Inverse time distance traveled per interrupt tick
  uint32_t d_per_tick;
  
  // Used by the stepper driver interrupt
  uint8_t execute_step;    // Flags step execution for each interrupt.
  uint8_t step_pulse_time; // Step pulse reset time after step rise
  uint8_t out_bits;        // The next stepping-bits to be output
  uint8_t load_flag;
  
  uint8_t ramp_count;
  uint8_t ramp_type;
} stepper_t;
static stepper_t st;

// Stores stepper buffer common data for a planner block. Data can change mid-block when the planner
// updates the remaining block velocity profile with a more optimal plan or a feedrate override occurs.
// NOTE: Normally, this buffer is only partially used, but can fill up completely in certain conditions.
typedef struct {  
  int32_t step_events_remaining;  // Tracks step event count for the executing planner block
  uint32_t d_next;                // Scaled distance to next step
  uint32_t initial_rate;          // Initialized step rate at re/start of a planner block
  uint32_t nominal_rate;          // The nominal step rate for this block in step_events/minute
  uint32_t rate_delta;            // The steps/minute to add or subtract when changing speed (must be positive)
  int32_t decelerate_after;
  float mm_per_step;
} st_data_t;
static st_data_t segment_data[SEGMENT_BUFFER_SIZE];

// Primary stepper buffer. Contains small, short line segments for the stepper algorithm to execute checked
// out incrementally from the first block in the planner buffer. These step segments 
typedef struct {
  uint8_t n_step;         // Number of step events to be executed for this segment
  uint8_t st_data_index;  // Stepper buffer common data index. Uses this information to execute this segment.
  uint8_t flag;           // Stepper algorithm execution flag to notify special conditions.
} st_segment_t;
static st_segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

static volatile uint8_t segment_buffer_tail;
static volatile uint8_t segment_buffer_head;
static uint8_t segment_next_head;

static volatile uint8_t busy;   // Used to avoid ISR nesting of the "Stepper Driver Interrupt". Should never occur though.
static plan_block_t *pl_current_block;  // A pointer to the planner block currently being traced
static st_segment_t *st_current_segment;
static st_data_t *st_current_data;

// Pointers for the step segment being prepped from the planner buffer. Accessed only by the
// main program. Pointers may be planning segments or planner blocks ahead of what being executed.
static plan_block_t *pl_prep_block;  // A pointer to the planner block being prepped into the stepper buffer
static uint8_t pl_prep_index;
static st_data_t *st_prep_data;
static uint8_t st_data_prep_index;

static uint8_t pl_partial_block_flag;



// Returns the index of the next block in the ring buffer
// NOTE: Removed modulo (%) operator, which uses an expensive divide and multiplication.
static uint8_t next_block_index(uint8_t block_index) 
{
  block_index++;
  if (block_index == SEGMENT_BUFFER_SIZE) { block_index = 0; }
  return(block_index);
}

static uint8_t next_block_pl_index(uint8_t block_index) 
{
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) { block_index = 0; }
  return(block_index);
}


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
   on the Pramod Ranade inverse time stepper algorithm, where a timer ticks at a constant
   frequency and uses time-distance counters to track when its the approximate time for any 
   step event. However, the Ranade algorithm, as described, is susceptible to numerical round-off,
   meaning that some axes steps may not execute for a given multi-axis motion.
     Grbl's algorithm slightly differs by using a single Ranade time-distance counter to manage
   a Bresenham line algorithm for multi-axis step events which ensures the number of steps for
   each axis are executed exactly. In other words, it uses a Bresenham within a Bresenham algorithm,
   where one tracks time(Ranade) and the other steps.
     This interrupt pops blocks from the block_buffer and executes them by pulsing the stepper pins
   appropriately. It is supported by The Stepper Port Reset Interrupt which it uses to reset the 
   stepper port after each pulse. The bresenham line tracer algorithm controls all three stepper
   outputs simultaneously with these two interrupts.
*/
// NOTE: Average time in this ISR is: 5 usec iterating timers only, 20-25 usec with step event, or
// 15 usec when popping a block. So, ensure Ranade frequency and step pulse times work with this.
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

      // NOTE: Loads after a step event. At high rates above 1/2 ISR frequency, there is 
      // a small chance that this will load at the same time as a step event. Hopefully,
      // the overhead for this loading event isn't too much.. possibly 2-5 usec.

      // NOTE: The stepper algorithm must control the planner buffer tail as it completes
      // the block moves. Otherwise, a feed hold can leave a few step buffer line moves 
      // without the correct planner block information.
      
      st_current_segment = &segment_buffer[segment_buffer_tail];
      
      // Load number of steps to execute from stepper buffer
      st.segment_steps_remaining = st_current_segment->n_step;
    
      // Check if the counters need to be reset for a new planner block
      if (st.load_flag == LOAD_BLOCK) {
        pl_current_block = plan_get_current_block(); // Should always be there. Stepper buffer handles this.
        st_current_data = &segment_data[segment_buffer[segment_buffer_tail].st_data_index]; //st_current_segment->st_data_index];
        
        // Initialize direction bits for block      
        st.out_bits = pl_current_block->direction_bits ^ settings.invert_mask;
        st.execute_step = true; // Set flag to set direction bits upon next ISR tick.
        
        // Initialize Bresenham line counters
        st.counter_x = (pl_current_block->step_event_count >> 1);
        st.counter_y = st.counter_x;
        st.counter_z = st.counter_x;
        
        // Initialize inverse time and step rate counter data
        st.counter_d = st_current_data->d_next;  // d_next always greater than delta_d.   
        if (st.delta_d < MINIMUM_STEP_RATE) { st.d_per_tick = MINIMUM_STEP_RATE; }
        else { st.d_per_tick = st.delta_d; }
                
        // During feed hold, do not update rate or ramp type. Keep decelerating.
//         if (sys.state == STATE_CYCLE) {
          st.delta_d = st_current_data->initial_rate; 
          st.ramp_count = ISR_TICKS_PER_ACCELERATION_TICK/2;  // Set ramp counter for trapezoid
          if (st.delta_d == st_current_data->nominal_rate) { st.ramp_type = RAMP_NOOP_CRUISE; }
          else { st.ramp_type = RAMP_ACCEL; }
//         }
      
      }

      // Acceleration and cruise handled by ramping. Just check if deceleration needs to begin.
      if (st_current_segment->flag == ST_DECEL || st_current_segment->flag == ST_DECEL_EOB) {
        if (st.ramp_type == RAMP_NOOP_CRUISE) {
          st.ramp_count = ISR_TICKS_PER_ACCELERATION_TICK/2;  // Set ramp counter for trapezoid
        } else {
          st.ramp_count = ISR_TICKS_PER_ACCELERATION_TICK-st.ramp_count; // Set ramp counter for triangle
        }
        st.ramp_type = RAMP_DECEL;
      }
      
      st.load_flag = LOAD_NOOP; // Motion loaded. Set no-operation flag until complete.

    } else {
      // Can't discard planner block here if a feed hold stops in middle of block.
      st_go_idle();
      bit_true(sys.execute,EXEC_CYCLE_STOP); // Flag main program for cycle end
      return; // Nothing to do but exit.
    }  
    
  } 
  
  // Adjust inverse time counter for ac/de-celerations
  // NOTE: Accelerations are handled by the stepper algorithm as it's thought to be more computationally
  // efficient on the Arduino AVR. This could change eventually, but it definitely will with ARM development.
  if (st.ramp_type) {
    st.ramp_count--; // Tick acceleration ramp counter
    if (st.ramp_count == 0) { // Adjust step rate when its time
      st.ramp_count = ISR_TICKS_PER_ACCELERATION_TICK; // Reload ramp counter
      if (st.ramp_type == RAMP_ACCEL) { // Adjust velocity for acceleration
        st.delta_d += st_current_data->rate_delta;
        if (st.delta_d >= st_current_data->nominal_rate) { // Reached nominal rate.
          st.delta_d = st_current_data->nominal_rate; // Set cruising velocity
          st.ramp_type = RAMP_NOOP_CRUISE; // Set ramp flag to ignore
        }
      } else { // Adjust velocity for deceleration
        if (st.delta_d > st_current_data->rate_delta) {
          st.delta_d -= st_current_data->rate_delta;
        } else {
        
          // Moving near zero feed rate. Gracefully slow down.
          st.delta_d >>= 1; // Integer divide by 2 until complete. Also prevents overflow.

          // Check for and handle feed hold exit? At this point, machine is stopped.

        }
      }
      // Finalize adjusted step rate. Ensure minimum.
      if (st.delta_d < MINIMUM_STEP_RATE) { st.d_per_tick = MINIMUM_STEP_RATE; }
      else { st.d_per_tick = st.delta_d; }
    }
  }
  
  // Iterate inverse time counter. Triggers each Bresenham step event.
  st.counter_d -= st.d_per_tick; 
  
  // Execute Bresenham step event, when it's time to do so.
  if (st.counter_d < 0) {  
    st.counter_d += st_current_data->d_next; // Reload inverse time counter

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
      if (st_current_segment->flag == ST_END_OF_BLOCK || st_current_segment->flag == ST_DECEL_EOB) {      
        plan_discard_current_block();
        st.load_flag = LOAD_BLOCK;
      } else {
        st.load_flag = LOAD_LINE;
      }
      
      // Discard current segment
      segment_buffer_tail = next_block_index( segment_buffer_tail );

      // NOTE: sys.position updates could be done here. The bresenham counters can have 
      // their own fast 8-bit addition-only counters. Here we would check the direction and 
      // apply it to sys.position accordingly. However, this could take too much time.
      
    }

    st.out_bits ^= settings.invert_mask;  // Apply step port invert mask    
  }
  busy = false;
// SPINDLE_ENABLE_PORT ^= 1<<SPINDLE_ENABLE_BIT;  
}


// The Stepper Port Reset Interrupt: Timer0 OVF interrupt handles the falling edge of the
// step pulse. This should always trigger before the next Timer2 COMPA interrupt and independently
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
//     st.ramp_count = ISR_TICKS_PER_ACCELERATION_TICK/2; 
//     st.delta_d = 0;
//     sys.state = STATE_QUEUED;
//   } else {
//     sys.state = STATE_IDLE;
//   }
    sys.state = STATE_IDLE;

}


/* Prepares step segment buffer. Continuously called from main program. 

   NOTE: There doesn't seem to be a great way to figure out how many steps occur within
   a set number of ISR ticks. Numerical round-off and CPU overhead always seems to be a
   critical problem. So, either numerical round-off checks could be made to account for 
   them, while CPU overhead could be minimized in some way, or we can flip the algorithm
   around to have the stepper algorithm track number of steps over an indeterminant amount
   of time instead. 
     In other words, we use the planner velocity floating point data to get an estimate of
   the number of steps we want to execute. We then back out the approximate velocity for
   the planner to use, which should be much more robust to round-off error. The main problem
   now is that we are loading the stepper algorithm to handle acceleration now, rather than
   pre-calculating with the main program. This approach does make sense in the way that
   planner velocities and stepper profiles can be traced more accurately. 
     Which is better? Very hard to tell. The time-based algorithm would be able to handle 
   Bresenham step adaptive-resolution much easier and cleaner. Whereas, the step-based would
   require some additional math in the stepper algorithm to adjust on the fly, plus adaptation
   would occur in a non-deterministic manner.
     I suppose it wouldn't hurt to build both to see what's better. Just a lot more work.

   TODO: Need to describe the importance of continuations of step pulses between ramp states
   and planner blocks. This has to do with Alden's problem with step "phase". The things I've
   been doing here limit this phase issue by truncating some of the ramp timing for certain
   events like deceleration initialization and end of block. 
*/

// !!! Need to make sure when a single partially completed block can be re-computed here with
//     new deceleration point and the segment manager begins accelerating again immediately.
void st_prep_buffer()
{
  while (segment_buffer_tail != segment_next_head) { // Check if we need to fill the buffer.
    
    // Determine if we need to load a new planner block.
    if (pl_prep_block == NULL) {
      pl_prep_block = plan_get_block_by_index(pl_prep_index); // Query planner for a queued block
      if (pl_prep_block == NULL) { return; } // No planner blocks. Exit.
      
      // Check if the planner has re-computed this block mid-execution. If so, push the old segment block
      // data Otherwise, prepare a new segment block data.
      if (pl_partial_block_flag) {
        
        // Prepare new shared segment block data and copy the relevant last segment block data.
        st_data_t *last_st_prep_data;
        last_st_prep_data = &segment_data[st_data_prep_index];
        st_data_prep_index = next_block_index(st_data_prep_index);
        st_prep_data = &segment_data[st_data_prep_index];
            
        st_prep_data->step_events_remaining = last_st_prep_data->step_events_remaining;
        st_prep_data->rate_delta = last_st_prep_data->rate_delta;
        st_prep_data->d_next = last_st_prep_data->d_next;
        st_prep_data->nominal_rate = last_st_prep_data->nominal_rate; // TODO: Recompute with feedrate overrides.
       
        st_prep_data->mm_per_step = last_st_prep_data->mm_per_step;

        pl_partial_block_flag = false; // Reset flag
        
        // TODO: If the planner updates this block, particularly from a deceleration to an acceleration,
        // we must reload the initial rate data, such that the velocity profile is re-constructed correctly.
        // The stepper algorithm must be flagged to adjust the acceleration counters.

      } else {
      
        // Prepare commonly shared planner block data for the ensuing segment buffer moves ad-hoc, since 
        // the planner buffer can dynamically change the velocity profile data as blocks are added.
        st_data_prep_index = next_block_index(st_data_prep_index);
        st_prep_data = &segment_data[st_data_prep_index];
      
        // Initialize Bresenham variables
        st_prep_data->step_events_remaining = pl_prep_block->step_event_count;

        // Convert planner block velocity profile data to stepper rate and step distance data.
        st_prep_data->nominal_rate = ceil(sqrt(pl_prep_block->nominal_speed_sqr)*(INV_TIME_MULTIPLIER/(60.0*ISR_TICKS_PER_SECOND))); // (mult*mm/isr_tic)
        st_prep_data->rate_delta = ceil(pl_prep_block->acceleration*
          ((INV_TIME_MULTIPLIER/(60.0*60.0))/(ISR_TICKS_PER_SECOND*ACCELERATION_TICKS_PER_SECOND))); // (mult*mm/isr_tic/accel_tic)
        st_prep_data->d_next = ceil((pl_prep_block->millimeters*INV_TIME_MULTIPLIER)/pl_prep_block->step_event_count); // (mult*mm/step)
        
        // TODO: Check if we really need to store this.
        st_prep_data->mm_per_step = pl_prep_block->millimeters/pl_prep_block->step_event_count;        
    
      }
      
      // Convert planner entry speed to stepper initial rate. 
      st_prep_data->initial_rate = ceil(sqrt(pl_prep_block->entry_speed_sqr)*(INV_TIME_MULTIPLIER/(60.0*ISR_TICKS_PER_SECOND))); // (mult*mm/isr_tic)

      // TODO: Nominal rate changes with feedrate override.
      // st_prep_data->nominal_rate = ceil(sqrt(pl_prep_block->nominal_speed_sqr)*(INV_TIME_MULTIPLIER/(60.0*ISR_TICKS_PER_SECOND))); // (mult*mm/isr_tic)

      // Calculate the planner block velocity profile type and determine deceleration point.
      float mm_decelerate_after = plan_calculate_velocity_profile(pl_prep_index);
      if (mm_decelerate_after == pl_prep_block->millimeters) {
        st_prep_data->decelerate_after = st_prep_data->step_events_remaining;
      } else {
        st_prep_data->decelerate_after = ceil( mm_decelerate_after/st_prep_data->mm_per_step );
      }
      
    }


    /* 
       TODO: Need to check for a planner flag to indicate a change to this planner block.
       If so, need to check for a change in acceleration state, from deceleration to acceleration,
       to reset the stepper ramp counters and the initial_rate data to trace the new
       ac/de-celeration profile correctly.
       No change conditions: 
         - From nominal speed to acceleration from feedrate override
         - From nominal speed to new deceleration.
         - From acceleration to new deceleration point later or cruising point.
         - From acceleration to immediate deceleration? Can happen during feedrate override
           and slowing down, but likely ok by enforcing the normal ramp counter protocol.
       Change conditions:
         - From deceleration to acceleration, i.e. common with jogging when new blocks are added.
    */

    st_segment_t *new_segment = &segment_buffer[segment_buffer_head];
    new_segment->st_data_index = st_data_prep_index;

    // TODO: How do you cheaply compute n_step without a sqrt()? Could be performed as 'bins'.
    // The basic equation is: s = u*t + 0.5*a*t^2
    // For the most part, we can store the acceleration portion in the st_data buffer and all
    // we would need to do is track the current approximate speed per loop with: v = u + a*t
    // Each loop would require 3 multiplication and 2 additions, since most of the variables 
    // are constants and would get compiled out.
    
//!!! Doesn't work as is. Requires last_velocity and acceleration in terms of steps, not mm.    
//     new_segment->n_step = ceil(last_velocity*TIME_PER_SEGMENT/mm_per_step);
//     if (st_prep_data->decelerate_after > 0) {
//       new_segment->n_step += ceil(pl_prep_block->acceleration*(0.5*TIME_PER_SEGMENT*TIME_PER_SEGMENT/(60*60))/mm_per_step);
//     } else {
//       new_segment->n_step -= ceil(pl_prep_block->acceleration*(0.5*TIME_PER_SEGMENT*TIME_PER_SEGMENT/(60*60))/mm_per_step);
//     }
    
    new_segment->n_step = 7; //floor( (exit_speed*approx_time)/mm_per_step );   
//      new_segment->n_step = max(new_segment->n_step,MINIMUM_STEPS_PER_BLOCK); // Ensure it moves for very slow motions?
//      new_segment->n_step = min(new_segment->n_step,MAXIMUM_STEPS_PER_BLOCK); // Prevent unsigned int8 overflow.

    
    // Check if n_step exceeds steps remaining in planner block. If so, truncate.
    if (new_segment->n_step > st_prep_data->step_events_remaining) { 
      new_segment->n_step = st_prep_data->step_events_remaining; 
      
      // Don't need to compute last velocity, since it will be refreshed with a new block.
    }

    // Check if n_step exceeds decelerate point in block. Need to perform this so that the
    // ramp counters are reset correctly in the stepper algorithm. Can be 1 step, but should
    // be OK since it is likely moving at a fast rate already.
    if (st_prep_data->decelerate_after > 0) {
      if (new_segment->n_step > st_prep_data->decelerate_after) { 
        new_segment->n_step = st_prep_data->decelerate_after; 
      }
// !!! Doesn't work. Remove if not using.
//       if (last_velocity < last_nominal_v) {
//       // !!! Doesn't work since distance changes and gets truncated.
//         last_velocity += pl_prep_block->acceleration*(TIME_PER_SEGMENT/(60*60)); // In acceleration ramp.
//         if {last_velocity > last_nominal_v) { last_velocity = last_nominal_v; } // Set to cruising.
//       }
//     } else { // In deceleration ramp
//       last_velocity -= pl_prep_block->acceleration*(TIME_PER_SEGMENT/(60*60));
    }
        
    // Update stepper block variables.
    st_prep_data->step_events_remaining -= new_segment->n_step;
    if ( st_prep_data->step_events_remaining == 0 ) { 
      // Move planner pointer to next block
      if (st_prep_data->decelerate_after == 0) {         
        new_segment->flag = ST_DECEL_EOB;  // Flag when deceleration begins and ends at EOB. Could rewrite to use bit flags too.
      } else {
        new_segment->flag = ST_END_OF_BLOCK;
      }
      pl_prep_index = next_block_pl_index(pl_prep_index);
      pl_prep_block = NULL;
    } else {
      // Current segment is mid-planner block. Just set the DECEL/NOOP acceleration flags.
      if (st_prep_data->decelerate_after == 0) {         
        new_segment->flag = ST_DECEL;
      } else {
        new_segment->flag = ST_NOOP;
      }
      st_prep_data->decelerate_after -= new_segment->n_step;
    }
    

    // New step segment completed. Increment segment buffer indices.
    segment_buffer_head = segment_next_head;
    segment_next_head = next_block_index(segment_buffer_head);   

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
  
  // TODO: NULL means that the segment buffer has completed the block. Need to clean this up a bit.
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
