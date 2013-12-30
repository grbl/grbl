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


// Some useful constants.
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
  uint32_t steps[N_AXIS];
  uint32_t step_event_count;
} st_block_t;
static st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE-1];
// TODO: Directly adjust this parameters to stop motion of individual axes for the homing cycle.
// But this may require this to be volatile if it is controlled by an interrupt.

// Primary stepper segment ring buffer. Contains small, short line segments for the stepper 
// algorithm to execute, which are "checked-out" incrementally from the first block in the
// planner buffer. Once "checked-out", the steps in the segments buffer cannot be modified by 
// the planner, where the remaining planner block steps still can.
typedef struct {
  uint16_t n_step;         // Number of step events to be executed for this segment
  uint8_t st_block_index; // Stepper block data index. Uses this information to execute this segment.
  uint16_t cycles_per_tick;  // Step distance traveled per ISR tick, aka step rate.
  uint8_t prescaler;
} segment_t;
static segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

// Stepper state variable. Contains running data and trapezoid variables.
typedef struct {
  // Used by the bresenham line algorithm
  uint32_t counter_x,        // Counter variables for the bresenham line tracer
          counter_y, 
          counter_z;

  #if STEP_PULSE_DELAY > 0
    uint8_t step_bits;  // Stores out_bits output to complete the step pulse delay
  #endif
  
  // Used by the stepper driver interrupt
  uint8_t execute_step;     // Flags step execution for each interrupt.
  uint8_t step_pulse_time;  // Step pulse reset time after step rise
  uint8_t out_bits;         // The next stepping-bits to be output

  uint16_t step_count;       // Steps remaining in line segment motion  
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

  float mm_per_step;
  float minimum_mm;
  
  uint8_t ramp_type;      // Current segment ramp state
  float mm_complete;      // End of velocity profile from end of current planner block in (mm).
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
  if (sys.state & (STATE_CYCLE | STATE_HOMING)){
    // Initialize stepper output bits
    st.out_bits = settings.invert_mask; 
        
    // Initialize step pulse timing from settings. Here to ensure updating after re-writing.
    #ifdef STEP_PULSE_DELAY
      // Set total step pulse time after direction pin set. Ad hoc computation from oscilloscope.
      st.step_pulse_time = -(((settings.pulse_microseconds+STEP_PULSE_DELAY-2)*TICKS_PER_MICROSECOND) >> 3);
      // Set delay between direction pin write and step command.
      OCR2A = -(((settings.pulse_microseconds)*TICKS_PER_MICROSECOND) >> 3);
    #else // Normal operation
      // Set step pulse time. Ad hoc computation from oscilloscope. Uses two's complement.
      st.step_pulse_time = -(((settings.pulse_microseconds-2)*TICKS_PER_MICROSECOND) >> 3);
    #endif

    // Enable stepper driver interrupt
    TIMSK1 |= (1<<OCIE1A);
  }
}


// Stepper shutdown
void st_go_idle() 
{
  // Disable stepper driver interrupt. Allow Timer0 to finish. It will disable itself.
  TIMSK1 &= ~(1<<OCIE1A); 
// TIMSK2 &= ~(1<<OCIE2A); // Disable Timer2 interrupt
// TCCR2B = 0; // Disable Timer2
  busy = false;

  // Disable steppers only upon system alarm activated or by user setting to not be kept enabled.
  if (((settings.stepper_idle_lock_time != 0xff) || bit_istrue(sys.execute,EXEC_ALARM)) && sys.state != STATE_HOMING) {
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


/* "The Stepper Driver Interrupt" - This timer interrupt is the workhorse of Grbl. Grbl employs
   the venerable Bresenham line algorithm to manage and exactly synchronize multi-axis moves.
   Unlike the popular DDA algorithm, the Bresenham algorithm is not susceptible to numerical
   round-off errors and only requires fast integer counters, meaning low computational overhead
   and maximizing the Arduino's capabilities. However, the downside of the Bresenham algorithm
   is, for certain multi-axis motions, the non-dominant axes may suffer from un-smooth step 
   pulse trains, which can lead to strange audible noises or shaking. This is particularly 
   noticeable or may cause motion issues at low step frequencies (<1kHz), but usually not at 
   higher frequencies.
     This interrupt is simple and dumb by design. All the computational heavy-lifting, as in
   determining accelerations, is performed elsewhere. This interrupt pops pre-computed segments,
   defined as constant velocity over n number of steps, from the step segment buffer and then 
   executes them by pulsing the stepper pins appropriately via the Bresenham algorithm. This 
   ISR is supported by The Stepper Port Reset Interrupt which it uses to reset the stepper port
   after each pulse. The bresenham line tracer algorithm controls all stepper outputs
   simultaneously with these two interrupts.
   
   NOTE: This interrupt must be as efficient as possible and complete before the next ISR tick, 
   which for Grbl must be less than 33.3usec (@30kHz ISR rate). Oscilloscope measured time in 
   ISR is 5usec typical and 25usec maximum, well below requirement.
*/
// TODO: Replace direct updating of the int32 position counters in the ISR somehow. Perhaps use smaller
// int8 variables and update position counters only when a segment completes. This can get complicated 
// with probing and homing cycles that require true real-time positions.
ISR(TIMER1_COMPA_vect)
{        
// SPINDLE_ENABLE_PORT ^= 1<<SPINDLE_ENABLE_BIT; // Debug: Used to time ISR
  if (busy) { return; } // The busy-flag is used to avoid reentering this interrupt
  
  // Set the direction pins a couple of nanoseconds before we step the steppers
  STEPPING_PORT = (STEPPING_PORT & ~DIRECTION_MASK) | (st.out_bits & DIRECTION_MASK);
  // Then pulse the stepping pins
  #ifdef STEP_PULSE_DELAY
    st.step_bits = (STEPPING_PORT & ~STEP_MASK) | st.out_bits; // Store out_bits to prevent overwriting.
  #else  // Normal operation
    STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | st.out_bits;
  #endif  

  // Enable step pulse reset timer so that The Stepper Port Reset Interrupt can reset the signal after
  // exactly settings.pulse_microseconds microseconds, independent of the main Timer1 prescaler.
  TCNT2 = st.step_pulse_time; // Reload timer counter
  TCCR2B = (1<<CS21); // Begin timer2. Full speed, 1/8 prescaler
  
  busy = true;
  sei(); // Re-enable interrupts to allow Stepper Port Reset Interrupt to fire on-time. 
         // NOTE: The remaining code in this ISR will finish before returning to main program.
    
  // If there is no step segment, attempt to pop one from the stepper buffer
  if (st.exec_segment == NULL) {
    // Anything in the buffer? If so, load and initialize next step segment.
    if (segment_buffer_head != segment_buffer_tail) {
      // Initialize new step segment and load number of steps to execute
      st.exec_segment = &segment_buffer[segment_buffer_tail];
      // Initialize step segment timing per step and load number of steps to execute.
      TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (st.exec_segment->prescaler<<CS10);
      OCR1A = st.exec_segment->cycles_per_tick;
      st.step_count = st.exec_segment->n_step; // NOTE: Can sometimes be zero when moving slow.
      // If the new segment starts a new planner block, initialize stepper variables and counters.
      // NOTE: When the segment data index changes, this indicates a new planner block.
      if ( st.exec_block_index != st.exec_segment->st_block_index ) {
        st.exec_block_index = st.exec_segment->st_block_index;
        st.exec_block = &st_block_buffer[st.exec_block_index];
        // Initialize Bresenham line and distance counters
        st.counter_x = (st.exec_block->step_event_count >> 1);
        st.counter_y = st.counter_x;
        st.counter_z = st.counter_x;
      }
    } else {
      // Segment buffer empty. Shutdown.
      st_go_idle();
      bit_true(sys.execute,EXEC_CYCLE_STOP); // Flag main program for cycle end
      return; // Nothing to do but exit.
    }  
  }
   
  // Reset out_bits and reload direction bits
  st.out_bits = st.exec_block->direction_bits; 

  // Execute step displacement profile by Bresenham line algorithm
  if (st.step_count > 0) {
    st.step_count--; // Decrement step events count 

    st.counter_x += st.exec_block->steps[X_AXIS];
    if (st.counter_x > st.exec_block->step_event_count) {
      st.out_bits |= (1<<X_STEP_BIT);
      st.counter_x -= st.exec_block->step_event_count;
      if (st.out_bits & (1<<X_DIRECTION_BIT)) { sys.position[X_AXIS]--; }
      else { sys.position[X_AXIS]++; }
    }
    st.counter_y += st.exec_block->steps[Y_AXIS];
    if (st.counter_y > st.exec_block->step_event_count) {
      st.out_bits |= (1<<Y_STEP_BIT);
      st.counter_y -= st.exec_block->step_event_count;
      if (st.out_bits & (1<<Y_DIRECTION_BIT)) { sys.position[Y_AXIS]--; }
      else { sys.position[Y_AXIS]++; }
    }
    st.counter_z += st.exec_block->steps[Z_AXIS];
    if (st.counter_z > st.exec_block->step_event_count) {
      st.out_bits |= (1<<Z_STEP_BIT);
      st.counter_z -= st.exec_block->step_event_count;
      if (st.out_bits & (1<<Z_DIRECTION_BIT)) { sys.position[Z_AXIS]--; }
      else { sys.position[Z_AXIS]++; }
    }  

    // During a homing cycle, lock out and prevent desired axes from moving.
    if (sys.state == STATE_HOMING) { st.out_bits &= sys.homing_axis_lock; }   
  }      

  if (st.step_count == 0) {
    // Segment is complete. Discard current segment and advance segment indexing.
    st.exec_segment = NULL;
    if ( ++segment_buffer_tail == SEGMENT_BUFFER_SIZE) { segment_buffer_tail = 0; }
  }

  st.out_bits ^= settings.invert_mask;  // Apply step port invert mask    
  busy = false;
// SPINDLE_ENABLE_PORT ^= 1<<SPINDLE_ENABLE_BIT;  
}


/* The Stepper Port Reset Interrupt: Timer0 OVF interrupt handles the falling edge of the step
   pulse. This should always trigger before the next Timer2 COMPA interrupt and independently
   finish, if Timer2 is disabled after completing a move. */
// ISR(TIMER0_OVF_vect)
// {
//   STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | (settings.invert_mask & STEP_MASK); 
//   TCCR0B = 0; // Disable timer until needed.
// }


// This interrupt is set up by ISR_TIMER1_COMPAREA when it sets the motor port bits. It resets
// the motor port after a short period (settings.pulse_microseconds) completing one step cycle.
// NOTE: Interrupt collisions between the serial and stepper interrupts can cause delays by
// a few microseconds, if they execute right before one another. Not a big deal, but can
// cause issues at high step rates if another high frequency asynchronous interrupt is 
// added to Grbl.
ISR(TIMER2_OVF_vect)
{
  // Reset stepping pins (leave the direction pins)
  STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | (settings.invert_mask & STEP_MASK); 
  TCCR2B = 0; // Disable Timer2 to prevent re-entering this interrupt when it's not needed. 
}


#ifdef STEP_PULSE_DELAY
  // This interrupt is used only when STEP_PULSE_DELAY is enabled. Here, the step pulse is
  // initiated after the STEP_PULSE_DELAY time period has elapsed. The ISR TIMER2_OVF interrupt
  // will then trigger after the appropriate settings.pulse_microseconds, as in normal operation.
  // The new timing between direction, step pulse, and step complete events are setup in the
  // st_wake_up() routine.
  ISR(TIMER2_COMPA_vect) 
  { 
    STEPPING_PORT = st.step_bits; // Begin step pulse.
  }
#endif


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

  // Configure Timer 1
  TCCR1B &= ~(1<<WGM13); // waveform generation = 0100 = CTC
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~(1<<WGM11); 
  TCCR1A &= ~(1<<WGM10);
  TCCR1A &= ~(3<<COM1A0); // output mode = 00 (disconnected)
  TCCR1A &= ~(3<<COM1B0); 
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (1<<CS10);

  // Configure Timer 2
  TCCR2A = 0; // Normal operation
  TCCR2B = 0; // Disable timer until needed.
  TIMSK2 |= (1<<TOIE2); // Enable Timer2 Overflow interrupt     
  #ifdef STEP_PULSE_DELAY
    TIMSK2 |= (1<<OCIE2A); // Enable Timer2 Compare Match A interrupt
  #endif

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
void st_prep_buffer()
{
  if (sys.state == STATE_QUEUED) { return; } // Block until a motion state is issued
  while (segment_buffer_tail != segment_next_head) { // Check if we need to fill the buffer.
    
    // Determine if we need to load a new planner block or if the block remainder is replanned. 
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
        st_prep_block->direction_bits = pl_block->direction_bits;
        st_prep_block->steps[X_AXIS] = pl_block->steps[X_AXIS];
        st_prep_block->steps[Y_AXIS] = pl_block->steps[Y_AXIS];
        st_prep_block->steps[Z_AXIS] = pl_block->steps[Z_AXIS];
        st_prep_block->step_event_count = pl_block->step_event_count;
        
        // Initialize segment buffer data for generating the segments.
        prep.steps_remaining = pl_block->step_event_count;
        prep.step_per_mm = prep.steps_remaining/pl_block->millimeters;
        prep.mm_per_step = pl_block->millimeters/prep.steps_remaining;
        prep.minimum_mm = pl_block->millimeters-prep.mm_per_step;

        if (sys.state == STATE_HOLD) {
          // Override planner block entry speed and enforce deceleration during feed hold.
          prep.current_speed = prep.exit_speed; 
          pl_block->entry_speed_sqr = prep.exit_speed*prep.exit_speed; 
        }
        else { prep.current_speed = sqrt(pl_block->entry_speed_sqr); }
      }
     
      /* --------------------------------------------------------------------------------- 
         Compute the velocity profile of a new planner block based on its entry and exit
         speeds, or recompute the profile of a partially-completed planner block if the 
         planner has updated it. For a commanded forced-deceleration, such as from a feed 
         hold, override the planner velocities and decelerate to the target exit speed.
      */
      prep.mm_complete = 0.0; // Default velocity profile complete at 0.0mm from end of block.
      float inv_2_accel = 0.5/pl_block->acceleration;
      if (sys.state == STATE_HOLD) {
        // Compute velocity profile parameters for a feed hold in-progress. This profile overrides
        // the planner block profile, enforcing a deceleration to zero speed.
        prep.ramp_type = RAMP_DECEL;
        float decel_dist = inv_2_accel*pl_block->entry_speed_sqr;
        if (decel_dist < pl_block->millimeters) {
          prep.exit_speed = 0.0;
          prep.mm_complete = pl_block->millimeters-decel_dist; // End of feed hold.
        } else {
          prep.exit_speed = sqrt(pl_block->entry_speed_sqr-2*pl_block->acceleration*pl_block->millimeters);
        }
      } else {                
        // Compute or recompute velocity profile parameters of the prepped planner block.
        prep.ramp_type = RAMP_ACCEL; // Initialize as acceleration ramp.
        prep.accelerate_until = pl_block->millimeters; 
        prep.exit_speed = plan_get_exec_block_exit_speed();   
        float exit_speed_sqr = prep.exit_speed*prep.exit_speed;
        float intersect_distance =
                0.5*(pl_block->millimeters+inv_2_accel*(pl_block->entry_speed_sqr-exit_speed_sqr));
        if (intersect_distance > 0.0) {
          if (intersect_distance < pl_block->millimeters) { // Either trapezoid or triangle types
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
            // prep.decelerate_after = pl_block->millimeters;
            prep.maximum_speed = prep.current_speed;
          }
        } else { // Acceleration-only type
          prep.accelerate_until = 0.0;
          // prep.decelerate_after = 0.0;
          prep.maximum_speed = prep.exit_speed;
        }
      }      
    }

    // Initialize new segment
    segment_t *prep_segment = &segment_buffer[segment_buffer_head];

    // Set new segment to point to the current segment data block.
    prep_segment->st_block_index = prep.st_block_index;

    /*------------------------------------------------------------------------------------
        Compute the average velocity of this new segment by determining the total distance
      traveled over the segment time DT_SEGMENT. The following code first attempts to create 
      a full segment based on the current ramp conditions. If the segment time is incomplete 
      when terminating at a ramp state change, the code will continue to loop through the
      progressing ramp states to fill the remaining segment execution time. However, if 
      an incomplete segment terminates at the end of the velocity profile, the segment is 
      considered completed despite having a truncated execution time less than DT_SEGMENT.
        The velocity profile is always assumed to progress through the ramp sequence:
      acceleration ramp, cruising state, and deceleration ramp. Each ramp's travel distance
      may range from zero to the length of the block. Velocity profiles can end either at 
      the end of planner block (typical) or mid-block at the end of a forced deceleration, 
      such as from a feed hold.
    */
    float dt_max = DT_SEGMENT;
    float dt = 0.0;
    float mm_remaining = pl_block->millimeters;
    float time_var = dt_max; // Time worker variable
    float mm_var; // mm-Distance worker variable
    float speed_var; // Speed worker variable.
    do {
      switch (prep.ramp_type) {
        case RAMP_ACCEL: 
          // NOTE: Acceleration ramp only computes during first do-while loop.
          speed_var = pl_block->acceleration*dt_max;
          mm_remaining -= dt_max*(prep.current_speed + 0.5*speed_var);
          if (mm_remaining < prep.accelerate_until) { // End of acceleration ramp.
            // Acceleration-cruise, acceleration-deceleration ramp junction, or end of block.
            mm_remaining = prep.accelerate_until; // NOTE: 0.0 at EOB
            time_var = 2.0*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            if (mm_remaining == prep.decelerate_after) { prep.ramp_type = RAMP_DECEL; }
            else { prep.ramp_type = RAMP_CRUISE; }
            prep.current_speed = prep.maximum_speed;
          } else { // Acceleration only. 
            prep.current_speed += speed_var;
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
          // NOTE: mm_var used as a misc worker variable to prevent errors when near zero speed.
          speed_var = pl_block->acceleration*time_var; // Used as delta speed (mm/min)
          if (prep.current_speed > speed_var) { // Check if at or below zero speed.
            // Compute distance from end of segment to end of block.
            mm_var = mm_remaining - time_var*(prep.current_speed - 0.5*speed_var); // (mm)
            if (mm_var > prep.mm_complete) { // Deceleration only.
              mm_remaining = mm_var;
              prep.current_speed -= speed_var;
              break; // Segment complete. Exit switch-case statement. Continue do-while loop.
            }
          } // End of block or end of forced-deceleration.
          time_var = 2.0*(mm_remaining-prep.mm_complete)/(prep.current_speed+prep.exit_speed);
          mm_remaining = prep.mm_complete; 
      }
      dt += time_var; // Add computed ramp time to total segment time.
      if (dt < dt_max) { time_var = dt_max - dt; } // **Incomplete** At ramp junction.
      else if (mm_remaining > prep.minimum_mm) { // Check for slow segments with zero steps.
        dt_max += DT_SEGMENT; // Increase segment time to ensure at least one step in segment.
        time_var = dt_max - dt;
      } else { break; } // **Complete** Exit loop. Segment execution time maxed.
    } while (mm_remaining > prep.mm_complete); // **Complete** Exit loop. Profile complete.
   
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
    uint32_t cycles;
    float steps_remaining = prep.step_per_mm*mm_remaining; 

    float inv_rate = dt/(prep.steps_remaining-steps_remaining);
    cycles = ceil( (TICKS_PER_MICROSECOND*1000000*60)*inv_rate ); // (ftol_mult*step/isr_tic)

    // Compute number of steps to execute and segment step phase correction. 
    prep_segment->n_step = ceil(prep.steps_remaining)-ceil(steps_remaining);

    // Determine end of segment conditions. Setup initial conditions for next segment.
    if (mm_remaining > prep.mm_complete) { 
      // Normal operation. Block incomplete. Distance remaining to be executed.
      prep.minimum_mm = prep.mm_per_step*floor(steps_remaining);
      pl_block->millimeters = mm_remaining;      
      prep.steps_remaining = steps_remaining;  
    } else { 
      // End of planner block or forced-termination. No more distance to be executed.
      if (mm_remaining > 0.0) { // At end of forced-termination.
        // NOTE: Currently only feed holds qualify for this scenario. May change with overrides.
        prep.current_speed = 0.0;
        prep.steps_remaining = ceil(steps_remaining);
        prep.minimum_mm = prep.steps_remaining-prep.mm_per_step;
        pl_block->millimeters = prep.steps_remaining*prep.mm_per_step; // Update with full steps.
        plan_cycle_reinitialize();
        sys.state = STATE_QUEUED; // End cycle.
      } else { // End of planner block
        // The planner block is complete. All steps are set to be executed in the segment buffer.
        pl_block = NULL;
        plan_discard_current_block();
      
        if (sys.state == STATE_HOLD) { 
          if (prep.current_speed == 0.0) { 
  // TODO: Check if the segment buffer gets initialized correctly.
            plan_cycle_reinitialize();
            sys.state = STATE_QUEUED; 
          }
        }
      }
    }

// TODO: Compile-time checks to what prescalers need to be compiled in.
    if (cycles < (1UL << 16)) { // < 65536  (4.1ms @ 16MHz)
      prep_segment->prescaler = 1; // prescaler: 0
      prep_segment->cycles_per_tick = cycles;
    } else {
      prep_segment->prescaler = 2; // prescaler: 8
      if (cycles < (1UL << 19)) { // < 524288 (32.8ms@16MHz)
        prep_segment->cycles_per_tick = cycles >> 3;
//     } else if (cycles < (1UL << 22)) { // < 4194304 (262ms@16MHz)
//       prep_segment->prescaler = 3; // prescaler: 64
//       prep_segment->cycles_per_tick =  cycles >> 6;
//     } else if (cycles < (1UL << 24)) { // < 16777216 (1.05sec@16MHz)
//       prep_segment->prescaler = 4; // prescaler: 256
//       prep_segment->cycles_per_tick =  (cycles >> 8);
//     } else {
//       prep_segment->prescaler = 5; // prescaler: 1024
//       if (cycles < (1UL << 26)) { // < 67108864 (4.19sec@16MHz)
//         prep_segment->cycles_per_tick = (cycles >> 10);
      } else { // Just set the slowest speed possible.
        prep_segment->cycles_per_tick = 0xffff;
      }
    }

    // New step segment initialization completed. Increment segment buffer indices.
    segment_buffer_head = segment_next_head;
    if ( ++segment_next_head == SEGMENT_BUFFER_SIZE ) { segment_next_head = 0; }

// int32_t blength = segment_buffer_head - segment_buffer_tail;
// if (blength < 0) { blength += SEGMENT_BUFFER_SIZE; } 
// printInteger(blength);
    
    if (sys.state & (STATE_QUEUED | STATE_HOMING)) { return; } // Force exit or one prepped segment.
  } 
}      

/* 
   TODO: With feedrate overrides, increases to the override value will not significantly
     change the current planner and stepper operation. When the value increases, we simply
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
       One "easy" way to do this is to have the step segment buffer enforce a deceleration and
     continually re-plan the planner buffer until the plan becomes feasible. This can work
     and may be easy to implement, but it expends a lot of CPU cycles and may block out the
     rest of the functions from operating at peak efficiency. Still the question is how do 
     we know when the plan is feasible in the context of what's already in the code and not
     require too much more code? 
*/
