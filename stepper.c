/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
  Part of Grbl

  Copyright (c) 2011-2012 Sungeun K. Jeon
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

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
   and Philipp Tiefenbacher. */

#include <avr/interrupt.h>
#include "stepper.h"
#include "config.h"
#include "settings.h"
#include "planner.h"

// Some useful constants
#define TICKS_PER_MICROSECOND (F_CPU/1000000)
#define CRUISE_RAMP 0
#define ACCEL_RAMP 1
#define DECEL_RAMP 2

// Stepper state variable. Contains running data and trapezoid variables.
typedef struct {
  // Used by the bresenham line algorithm
  int32_t counter_x,        // Counter variables for the bresenham line tracer
          counter_y, 
          counter_z;
  uint32_t event_count;     // Total event count. Retained for feed holds. 
  uint32_t step_events_remaining;  // Steps remaining in motion

  // Used by Pramod Ranade inverse time algorithm
  int32_t delta_d;     // Ranade distance traveled per interrupt tick
  int32_t d_counter;   // Ranade distance traveled since last step event
  uint8_t ramp_count; // Acceleration interrupt tick counter. 
  uint8_t ramp_type; // Ramp type variable.
  uint8_t execute_step; // Flags step execution for each interrupt.
  
} stepper_t;
static stepper_t st;
static block_t *current_block;  // A pointer to the block currently being traced

// Used by the stepper driver interrupt
static uint8_t step_pulse_time; // Step pulse reset time after step rise
static uint8_t out_bits;        // The next stepping-bits to be output

// NOTE: If the main interrupt is guaranteed to be complete before the next interrupt, then
// this blocking variable is no longer needed. Only here for safety reasons.
static volatile uint8_t busy;   // True when "Stepper Driver Interrupt" is being serviced. Used to avoid retriggering that handler.

//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
// 
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates by block->rate_delta
//  until reaching cruising speed block->nominal_rate, and/or until step_events_remaining reaches block->decelerate_after
//  after which it decelerates until the block is completed. The driver uses constant acceleration, which is applied as
//  +/- block->rate_delta velocity increments by the midpoint rule at each ACCELERATION_TICKS_PER_SECOND.

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
    out_bits = settings.invert_mask; 
    // Initialize step pulse timing from settings.
    step_pulse_time = -(((settings.pulse_microseconds-2)*TICKS_PER_MICROSECOND) >> 3);
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


// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse of Grbl. It is based
// on the Pramod Ranade inverse time stepper algorithm, where a timer ticks at a constant
// frequency and uses time-distance counters to track when its the approximate time for any 
// step event. However, the Ranade algorithm, as described, is susceptible to numerical round-off,
// meaning that some axes steps may not execute for a given multi-axis motion.
//   Grbl's algorithm slightly differs by using a single Ranade time-distance counter to manage
// a Bresenham line algorithm for multi-axis step events which ensures the number of steps for
// each axis are executed exactly. In other words, it uses a Bresenham within a Bresenham algorithm,
// where one tracks time(Ranade) and the other steps.
//   This interrupt pops blocks from the block_buffer and executes them by pulsing the stepper pins
// appropriately. It is supported by The Stepper Port Reset Interrupt which it uses to reset the 
// stepper port after each pulse. The bresenham line tracer algorithm controls all three stepper
// outputs simultaneously with these two interrupts.
//
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
    STEPPING_PORT = ( STEPPING_PORT & ~(DIRECTION_MASK | STEP_MASK) ) | out_bits;
    TCNT0 = step_pulse_time; // Reload Timer0 counter.
    TCCR0B = (1<<CS21); // Begin Timer0. Full speed, 1/8 prescaler
  }
  
  busy = true;
  sei(); // Re-enable interrupts. This ISR will still finish before returning to main program.
  
  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
  
    // Anything in the buffer? If so, initialize next motion.
    current_block = plan_get_current_block();
    if (current_block != NULL) {
      // By algorithm design, the loading of the next block never coincides with a step event,
      // since there is always one Ranade timer tick before a step event occurs. This means
      // that the Bresenham counter math never is performed at the same time as the loading 
      // of a block, hence helping minimize total time spent in this interrupt.

      // Initialize direction bits for block      
      out_bits = current_block->direction_bits ^ settings.invert_mask;
      st.execute_step = true; // Set flag to set direction bits.
      
      // Initialize Bresenham variables
      st.counter_x = (current_block->step_event_count >> 1);
      st.counter_y = st.counter_x;
      st.counter_z = st.counter_x;
      st.event_count = current_block->step_event_count;
      st.step_events_remaining = st.event_count;
      
      // During feed hold, do not update Ranade counter, rate, or ramp type. Keep decelerating.
      if (sys.state == STATE_CYCLE) {
        // Initialize Ranade variables
        st.d_counter = current_block->d_next;  
        st.delta_d = current_block->initial_rate;
        st.ramp_count = ISR_TICKS_PER_ACCELERATION_TICK/2;
        
        // Initialize ramp type.
        if (st.step_events_remaining == current_block->decelerate_after) { st.ramp_type = DECEL_RAMP; }
        else if (st.delta_d == current_block->nominal_rate) { st.ramp_type = CRUISE_RAMP; }
        else { st.ramp_type = ACCEL_RAMP; }
      }
      
    } else {
      st_go_idle();
      bit_true(sys.execute,EXEC_CYCLE_STOP); // Flag main program for cycle end
      busy = false;
      return; // Nothing to do but exit.
    }  
  } 
  
  // Adjust inverse time counter for ac/de-celerations
  if (st.ramp_type) {
    // Tick acceleration ramp counter
    st.ramp_count--;
    if (st.ramp_count == 0) {
      st.ramp_count = ISR_TICKS_PER_ACCELERATION_TICK; // Reload ramp counter
      if (st.ramp_type == ACCEL_RAMP) { // Adjust velocity for acceleration
        st.delta_d += current_block->rate_delta;
        if (st.delta_d >= current_block->nominal_rate) { // Reached cruise state.
          st.ramp_type = CRUISE_RAMP;
          st.delta_d = current_block->nominal_rate; // Set cruise velocity
        }
      } else if (st.ramp_type == DECEL_RAMP) { // Adjust velocity for deceleration
        if (st.delta_d > current_block->rate_delta) {
          st.delta_d -= current_block->rate_delta;
        } else {
          st.delta_d >>= 1; // Integer divide by 2 until complete. Also prevents overflow.
        }
      }
    }
  }
    
  // Iterate Pramod Ranade inverse time counter. Triggers each Bresenham step event.
  if (st.delta_d < MINIMUM_STEP_RATE) { st.d_counter -= MINIMUM_STEP_RATE; }
  else { st.d_counter -= st.delta_d; }
  
  // Execute Bresenham step event, when it's time to do so.
  if (st.d_counter < 0) {  
    st.d_counter += current_block->d_next;
    
    // Check for feed hold state and execute accordingly.
    if (sys.state == STATE_HOLD) {
      if (st.ramp_type != DECEL_RAMP) {
        st.ramp_type = DECEL_RAMP;
        st.ramp_count = ISR_TICKS_PER_ACCELERATION_TICK/2; 
      } 
      if (st.delta_d <= current_block->rate_delta) {
        st_go_idle();
        bit_true(sys.execute,EXEC_CYCLE_STOP);
        busy = false;
        return;
      }
    }
    
    // TODO: Vary Bresenham resolution for smoother motions or enable faster step rates (>20kHz).
    
    out_bits = current_block->direction_bits; // Reset out_bits and reload direction bits
    st.execute_step = true;
    
    // Execute step displacement profile by Bresenham line algorithm
    st.counter_x -= current_block->steps_x;
    if (st.counter_x < 0) {
      out_bits |= (1<<X_STEP_BIT);
      st.counter_x += st.event_count;
      if (out_bits & (1<<X_DIRECTION_BIT)) { sys.position[X_AXIS]--; }
      else { sys.position[X_AXIS]++; }
    }
    st.counter_y -= current_block->steps_y;
    if (st.counter_y < 0) {
      out_bits |= (1<<Y_STEP_BIT);
      st.counter_y += st.event_count;
      if (out_bits & (1<<Y_DIRECTION_BIT)) { sys.position[Y_AXIS]--; }
      else { sys.position[Y_AXIS]++; }
    }
    st.counter_z -= current_block->steps_z;
    if (st.counter_z < 0) {
      out_bits |= (1<<Z_STEP_BIT);
      st.counter_z += st.event_count;
      if (out_bits & (1<<Z_DIRECTION_BIT)) { sys.position[Z_AXIS]--; }
      else { sys.position[Z_AXIS]++; }
    }

    // Check step events for trapezoid change or end of block.
    st.step_events_remaining--; // Decrement step events count
    if (st.step_events_remaining) {
      if (st.ramp_type != DECEL_RAMP) {
        // Acceleration and cruise handled by ramping. Just check for deceleration.
        if (st.step_events_remaining <=  current_block->decelerate_after) {
          st.ramp_type = DECEL_RAMP;
          if (st.step_events_remaining == current_block->decelerate_after) {
            if (st.delta_d == current_block->nominal_rate) {
              st.ramp_count = ISR_TICKS_PER_ACCELERATION_TICK/2;  // Set ramp counter for trapezoid
            } else {
              st.ramp_count = ISR_TICKS_PER_ACCELERATION_TICK-st.ramp_count; // Set ramp counter for triangle
            }
          }
        }
      }
    } else {
      // If current block is finished, reset pointer 
      current_block = NULL;
      plan_discard_current_block();
    }

    out_bits ^= settings.invert_mask;  // Apply step port invert mask    
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
  current_block = NULL;
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
