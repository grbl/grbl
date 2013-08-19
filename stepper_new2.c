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
#define CRUISE_RAMP 0
#define ACCEL_RAMP 1
#define DECEL_RAMP 2

#define LOAD_NOOP 0
#define LOAD_LINE 1
#define LOAD_BLOCK 2

// Stepper state variable. Contains running data and trapezoid variables.
typedef struct {
  // Used by the bresenham line algorithm
  int32_t counter_x,        // Counter variables for the bresenham line tracer
          counter_y, 
          counter_z;
  int32_t step_events_remaining;  // Steps remaining in line motion

  // Used by inverse time algorithm
  int32_t delta_d;      // Inverse time distance traveled per interrupt tick
  int32_t d_counter;    // Inverse time distance traveled since last step event
  int32_t d_per_tick;
  
  // Used by the stepper driver interrupt
  uint8_t execute_step;    // Flags step execution for each interrupt.
  uint8_t step_pulse_time; // Step pulse reset time after step rise
  uint8_t out_bits;        // The next stepping-bits to be output
  uint8_t load_flag;
} stepper_t;
static stepper_t st;

#define STEPPER_BUFFER_SIZE 5
typedef struct {
  int32_t event_count;
  int32_t rate;
  uint8_t end_of_block;
  uint8_t tick_count;

  int32_t initial_rate;     // The step rate at start of block  
  int32_t rate_delta;       // The steps/minute to add or subtract when changing speed (must be positive)
  int32_t decelerate_after; // The index of the step event on which to start decelerating
  int32_t nominal_rate;     // The nominal step rate for this block in step_events/minute
  int32_t d_next;           // Scaled distance to next step

} stepper_buffer_t;
static stepper_buffer_t step_buffer[STEPPER_BUFFER_SIZE];

static volatile uint8_t step_buffer_tail;
static uint8_t step_buffer_head;
static uint8_t step_next_head;

// NOTE: If the main interrupt is guaranteed to be complete before the next interrupt, then
// this blocking variable is no longer needed. Only here for safety reasons.
static volatile uint8_t busy;   // True when "Stepper Driver Interrupt" is being serviced. Used to avoid retriggering that handler.
static plan_block_t *plan_current_block;  // A pointer to the planner block currently being traced


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
  
  // This loading step is important. Allows the stepper ISR to only handle its own variables
  // hence no volatiles needed. Otherwise a preloading step is required by the main program
  // or some other means to get the line motions started, and volatile would be required.
  
  // If there is no current block, attempt to pop one from the buffer
  if (st.load_flag != LOAD_NOOP) {
    
    // Anything in the buffer? If so, initialize next motion.
    if (step_buffer_head != step_buffer_tail) {

      // NOTE: Loads after a step event. At high rates above 1/2 ISR frequency, there is 
      // a small chance that this will load at the same time as a step event. Hopefully,
      // the overhead for this loading event isn't too much.. possibly 2-5 usec.

      // NOTE: The stepper algorithm must control the planner buffer tail as it completes
      // the block moves. Otherwise, a feed hold can leave a few step buffer line moves 
      // without the correct planner block information.

      // Load line motion from stepper buffer
      st.step_events_remaining = step_buffer[step_buffer_tail].event_count;
      st.delta_d = step_buffer[step_buffer_tail].rate; 
    
      // Check if the counters need to be reset
      if (st.load_flag == LOAD_BLOCK) {
        plan_current_block = plan_get_current_block();
        
        // Initialize direction bits for block      
        st.out_bits = plan_current_block->direction_bits ^ settings.invert_mask;
        st.execute_step = true; // Set flag to set direction bits.

        st.counter_x = (plan_current_block->step_event_count >> 1);
        st.counter_y = st.counter_x;
        st.counter_z = st.counter_x;
        
        // This is correct. Sets the total time before the next step occurs.
        st.counter_d = plan_current_block->d_next;  // d_next always greater than delta_d.   
        
        st.ramp_count = ISR_TICKS_PER_ACCELERATION_TICK/2;  // Set ramp counter for trapezoid
        
      }
      
      st.load_flag = LOAD_NOOP; // Line motion loaded. Set no-operation flag until complete.

    } else {
      // Can't discard planner block here if a feed hold stops in middle of block.
      st_go_idle();
      bit_true(sys.execute,EXEC_CYCLE_STOP); // Flag main program for cycle end
      return; // Nothing to do but exit.
    }  
    
  } 
  
  // Iterate inverse time counter. Triggers each Bresenham step event.
  st.counter_d -= st.delta_d; 
  
  // Execute Bresenham step event, when it's time to do so.
  if (st.counter_d < 0) {  
    st.counter_d += plan_current_block->d_next; // Reload inverse time counter
    
    st.out_bits = plan_current_block->direction_bits; // Reset out_bits and reload direction bits
    st.execute_step = true;
    
    // Execute step displacement profile by Bresenham line algorithm
    st.counter_x -= plan_current_block->steps[X_AXIS];
    if (st.counter_x < 0) {
      st.out_bits |= (1<<X_STEP_BIT);
      st.counter_x += plan_current_block->step_event_count;
      if (st.out_bits & (1<<X_DIRECTION_BIT)) { sys.position[X_AXIS]--; }
      else { sys.position[X_AXIS]++; }
    }
    st.counter_y -= plan_current_block->steps[Y_AXIS];
    if (st.counter_y < 0) {
      st.out_bits |= (1<<Y_STEP_BIT);
      st.counter_y += plan_current_block->step_event_count;
      if (st.out_bits & (1<<Y_DIRECTION_BIT)) { sys.position[Y_AXIS]--; }
      else { sys.position[Y_AXIS]++; }
    }
    st.counter_z -= plan_current_block->steps[Z_AXIS];
    if (st.counter_z < 0) {
      st.out_bits |= (1<<Z_STEP_BIT);
      st.counter_z += plan_current_block->step_event_count;
      if (st.out_bits & (1<<Z_DIRECTION_BIT)) { sys.position[Z_AXIS]--; }
      else { sys.position[Z_AXIS]++; }
    }

    // Check step events for trapezoid change or end of block.
    st.step_events_remaining--; // Decrement step events count
    
    // Tracking step events may not be the right thing. Need to track time instead.
    // If a step completes and the inverse counters are reset, then the line motion time
    // to execute gets truncated. Thus, screwing up the acceleration.
    // Time can change at the first block, a transition point or end of block. It is not constant.
    
    // Time must be truncated when the last step in the block is executed. Ensures there are 
    // no step phase issues with the following block. Or, the ISR timer count must be computed
    // to be exact, such that the last step occurs correctly.

    // This should always/automatically occur for a true trapezoid. The first acceleration
    // should be 1/2 accel_tick and the last decel should also. The difference is taken up by
    // the transition from cruise to decel.

    // For a cruising entry, the transition block takes care of incompatibility.
    // For a cruising exit, the exit block needs to truncate. Especially when the cruising
    // continues onward to the next block.
    
    if (st.step_events_remaining == 0) {

      // Line move is complete, set load line flag to check for new move.
      // Check if last line move in planner block. Discard if so.
      if (step_buffer[step_buffer_tail].end_of_block) {      
        plan_discard_current_block();
        st.load_flag = LOAD_BLOCK;
      } else {
        st.load_flag = LOAD_LINE;
      }
      
      // Discard current block
      if (step_buffer_head != step_buffer_tail) { 
        step_buffer_tail = next_block_index( step_buffer_tail );
      }

      // NOTE: sys.position updates could be done here. The bresenham counters can have 
      // their own incremental counters. Here we would check the direction and apply it
      // to sys.position accordingly. However, this could take some time.
      
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
  if (current_block != NULL) {
    // Replan buffer from the feed hold stop location.
    plan_cycle_reinitialize(st.step_events_remaining);
    st.ramp_type = ACCEL_RAMP;
    st.ramp_count = ISR_TICKS_PER_ACCELERATION_TICK/2; 
    st.delta_d = 0;
    sys.state = STATE_QUEUED;
  } else {
    sys.state = STATE_IDLE;
  }
}


// Preps stepper buffer. Called from main program. Sends short line segments of constant
// velocity to the stepper driver. Ac/de-celeration calculations are performed here.
// NOTE: Line motions can range from 1-300
// TODO: Could be very easy to install adaptive Bresenham resolution with this prep function.\
         That is, if the acceleration calculations are performed here rather than in the ISR.
         
void st_prep_buffer()
{
   while (st.buffer_tail != st.next_head) { // Check if we need to fill the buffer.

     step_block_t *step_block = &step_buffer[st.buffer_head];

     // Determine if we need to load a new planner block.
     if (plan_current_block == NULL) {
       plan_current_block = plan_get_current_block();
       if (plan_current_block == NULL) { return; } // No more planner blocks. Let stepper finish out.
              
       // Initialize Bresenham variables
       step_buffer[st.buffer_head] = plan_current_block->step_event_count;
       //st.step_events_remaining = st.event_count;
       
       // Convert new block to stepper variables.
       // NOTE: This data can change mid-block from normal planner updates and feedrate overrides. Must
       // be maintained as these execute.
       // TODO: The initial rate needs to be sent back to the planner to update the entry speed
       block->initial_rate = ceil(sqrt(plan_current_block->entry_speed_sqr)*(INV_TIME_MULTIPLIER/(60*ISR_TICKS_PER_SECOND))); // (mult*mm/isr_tic)
       block->nominal_rate = ceil(plan_current_block->nominal_speed*(INV_TIME_MULTIPLIER/(60.0*ISR_TICKS_PER_SECOND))); // (mult*mm/isr_tic)

       // This data doesn't change. Could be performed in the planner, but fits nicely here.
       // Although, acceleration can change for S-curves. So keep it here.
       block->rate_delta = ceil(plan_current_block->acceleration*
         ((INV_TIME_MULTIPLIER/(60.0*60.0))/(ISR_TICKS_PER_SECOND*ACCELERATION_TICKS_PER_SECOND))); // (mult*mm/isr_tic/accel_tic)
       // This definitely doesn't change, but could be precalculated in a way to help some of the 
       // math in this handler, i.e. millimeters per step event data.
       block->d_next = ceil((plan_current_block->millimeters*INV_TIME_MULTIPLIER)/plan_current_block->step_event_count); // (mult*mm/step)
       
       // During feed hold, do not update Ranade counter, rate, or ramp type. Keep decelerating.
       if (sys.state == STATE_CYCLE) {       }
     }
 
 
// Track instead the trapezoid line and use the average of the entry and exit velocities
// to determine step rate. This should take care of the deceleration issue automatically... 
// i think.
// First need to figure out what type of profile is segment is, i.e. acceleration only, accel
// to decel triangle, cruise to decel, or all three. May need more profile data to compute this
// from the planner itself, like accelerate until.
// Another issue. This is only tracking the velocity profile, not the distance covered over that
// time period. This can lead to an unsynchronized velocity profile and steps executed. But,
// how much drift is there really? Enough to be a problem? Not sure. I would think typical 
// drift would be on the order of a few steps, more depending on step resolution. 
entry_rate = last_exit_rate;
time = 250 ISR ticks per acceleration tick.
distance = 0;
if (distance_traveled < accelerate_until)
  exit_rate = entry_rate + acceleration*time;
  if (exit_rate > nominal_rate) {
    exit_rate = nominal_rate;
    time = 2*(accelerate_until-distance_travel)/(entry_rate+nominal_rate);
    // distance = accelerate_until; // Enforce distance?
    // Truncate this segment.
  }
} else if (distance_traveled >= decelerate_after) {
  if (accelerate_until == decelerate_after) {
    time = last time;
    exit_rate = entry_rate;
  } else {
    exit_rate = entry_rate - acceleration*time;
  }
} else {  
  exit_rate = nominal_rate; // Just cruise
  distance = nominal_rate*time;
  if (distance > decelerate_after) { // Truncate segment at nominal rate.
    time = (decelerate_after-distance_traveled)/(nominal_rate);
    distance = decelerate_after;
  }
}
mean_rate = 0.5*(entry_rate+exit_rate);
distance = mean_rate*time;
  
   
if (entry_rate < nominal_rate) {  
  if (entry_distance < decelerate_after) { // Acceleration case
    exit_rate = entry_rate + acceleration*time
    exit_rate = min(exit_rate,nominal_rate);
mean_rate = 0.5*(entry_rate + exit_rate);
distance = mean_rate*time;
if (distance > decelerate_after) {
  exit_rate = 

 
 // If the MINIMUM_STEP_RATE is less than ACCELERATION_TICKS_PER_SECOND then there can be
 // rate adjustements that have less than one step per tick.
 // How do you deal with the remainer?
time = 250 ISR ticks per acceleration tick. (30000/120)
delta_d*time // mm per acceleration tick
delta_d*time/d_next // number of steps/acceleration_tick. Chance of integer overflow.
delta_d*time/d_next + last_remainder. // steps/acceleration_tick.
n_step*d_next/delta_d // number of ISR ticks for enforced n_steps.

// In floating point? Then convert?
// Requires exact millimeters. Roundoff might be a problem. But could be corrected by just
// checking if the total step event counts are performed.
// Could be limited by float conversion and about 1e7 steps per block.
line_mm = feed_rate / acc_tick // mm per acc_tick
n_steps = floor(line_mm * step_event_remaining/millimeters_remaining) // steps. float 7.2 digits|int32 10 digits
millimeters_remaining -= line_mm; 
step_events_remaining -= n_steps;

// There doesn't seem to be a way to avoid this divide here. 
line_mm = feed_rate / acc_tick // mm per acc_tick
n_steps = floor( (line_mm+line_remainder) * step_event_count/millimeters) // steps. float 7.2 digits|int32 10 digits
line_remainder = line_mm - n_steps*(millimeters/step_event_count);

// Need to handle when rate is very very low, i.e. less than one step per accel tick.
// Could be bounded by MINIMUM_STEP_RATE.

// 1. Figure out how many steps occur exactly within n ISR ticks.
// 2. Account for step-time remainder for next line motion exactly.
// 3. At the end of block, determine exact number of ISR ticks to finish the steps. Or,\
       have the ISR track steps to exit on time. It would require an extra counter.

// NOTE: There doesn't seem to be a great way to figure out how many steps occur within
// a set number of ISR ticks. Numerical round-off and CPU overhead always seems to be a
// critical problem. So, either numerical round-off checks could be made to account for 
// them, while CPU overhead could be minimized in some way, or we can flip the algorithm
// around to have the stepper algorithm track number of steps over an indeterminant amount
// of time instead. 
//   In other words, we use the planner velocity floating point data to get an estimate of
// the number of steps we want to execute. We then back out the approximate velocity for
// the planner to use, which should be much more robust to round-off error. The main problem
// now is that we are loading the stepper algorithm to handle acceleration now, rather than
// pre-calculating with the main program. This approach does make sense in the way that
// planner velocities and stepper profiles can be traced more accurately. 
//   Which is better? Very hard to tell. The time-based algorithm would be able to handle 
// Bresenham step adaptive-resolution much easier and cleaner. Whereas, the step-based would
// require some additional math in the stepper algorithm to adjust on the fly, plus adaptation
// would occur in a non-deterministic manner.
//   I suppose it wouldn't hurt to build both to see what's better. Just a lot more work.

feed_rate/120 = millimeters per acceleration tick


     steps?
     d_next // (mult*mm/step)
     rate   // (mult*mm/isr_tic)
     rate/d_next  // step/isr_tic

     if (plan_current_block->step_events_remaining <= plan_current_block->decelerate_after) {
       // Determine line segment velocity and associated inverse time counter.
       if (step_block.ramp_type == ACCEL_RAMP) { // Adjust velocity for acceleration
         step_block.delta_d += plan_current_block->rate_delta;
         if (step_block.delta_d >= plan_current_block->nominal_rate) { // Reached cruise state.
           step_block.ramp_type = CRUISE_RAMP;
           step_block.delta_d = plan_current_block->nominal_rate; // Set cruise velocity
         }
       } 
     } else { // Adjust velocity for deceleration
       if (step_block.delta_d > plan_current_block->rate_delta) {
         step_block.delta_d -= plan_current_block->rate_delta;
       } else {
         step_block.delta_d >>= 1; // Integer divide by 2 until complete. Also prevents overflow.
       }
     }
      
     // Incorrect. Can't overwrite delta_d. Needs to override instead.
     if (step_block.delta_d < MINIMUM_STEP_RATE) { step_block.delta_d = MINIMUM_STEP_RATE; }

     /* - Compute the number of steps needed to complete this move over the move time, i.e.
          ISR_TICKS_PER_ACCELERATION_TICK.
          - The first block in the buffer is half of the move time due to midpoint rule.
        - Check if this reaches the deceleration after location. If so, truncate move. Also,
          if this is a triangle move, double the truncated move to stay with midpoint rule.
          NOTE: This can create a stepper buffer move down to just one step in length. 
        - Update the planner block entry speed for the planner to compute from end of the
          stepper buffer location.
        - If a feed hold occurs, begin to enforce deceleration, while enforcing the above rules.
          When the deceleration is complete, all we need to do is update the planner block
          entry speed and force a replan.
     */    
 
     // Planner block move completed.
     // TODO: planner buffer tail no longer needs to be volatile. only accessed by main program.
     if (st.step_events_remaining == 0) { 
       plan_current_block = NULL; // Set flag that we are done with this planner block.
       plan_discard_current_block();
     }
     

  
  
     
     step_buffer_head = step_next_head;
     step_next_head = next_block_index(step_buffer_head); 
      
   }      
}      
