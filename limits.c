/*
  limits.c - code pertaining to limit-switches and performing the homing cycle
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2012-2013 Sungeun K. Jeon

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
  
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "stepper.h"
#include "settings.h"
#include "nuts_bolts.h"
#include "config.h"
#include "spindle_control.h"
#include "motion_control.h"
#include "planner.h"
#include "protocol.h"
#include "limits.h"
#include "report.h"


void limits_init() 
{
  LIMIT_DDR &= ~(LIMIT_MASK); // Set as input pins

  #ifndef LIMIT_SWITCHES_ACTIVE_HIGH
    LIMIT_PORT |= (LIMIT_MASK); // Enable internal pull-up resistors. Normal high operation.
  #else
    LIMIT_PORT &= ~(LIMIT_MASK); // Normal low operation. Requires external pull-down.
  #endif

  if (bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE)) {
    LIMIT_PCMSK |= LIMIT_MASK; // Enable specific pins of the Pin Change Interrupt
    PCICR |= (1 << LIMIT_INT); // Enable Pin Change Interrupt
  } else {
    limits_disable(); 
  }
  
  #ifdef ENABLE_SOFTWARE_DEBOUNCE
    MCUSR &= ~(1<<WDRF);
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    WDTCSR = (1<<WDP0);
  #endif
}


void limits_disable()
{
  LIMIT_PCMSK &= ~LIMIT_MASK;  // Disable specific pins of the Pin Change Interrupt
  PCICR &= ~(1 << LIMIT_INT);  // Disable Pin Change Interrupt
}


// This is the Limit Pin Change Interrupt, which handles the hard limit feature. A bouncing 
// limit switch can cause a lot of problems, like false readings and multiple interrupt calls.
// If a switch is triggered at all, something bad has happened and treat it as such, regardless
// if a limit switch is being disengaged. It's impossible to reliably tell the state of a 
// bouncing pin without a debouncing method.
// NOTE: Do not attach an e-stop to the limit pins, because this interrupt is disabled during
// homing cycles and will not respond correctly. Upon user request or need, there may be a
// special pinout for an e-stop, but it is generally recommended to just directly connect
// your e-stop switch to the Arduino reset pin, since it is the most correct way to do this.
#ifdef ENABLE_SOFTWARE_DEBOUNCE
  ISR(LIMIT_INT_vect) { if (!(WDTCSR & (1<<WDIE))) { WDTCSR |= (1<<WDIE); } }
  ISR(WDT_vect) 
  {
    WDTCSR &= ~(1<<WDIE); 
    // Ignore limit switches if already in an alarm state or in-process of executing an alarm.
    // When in the alarm state, Grbl should have been reset or will force a reset, so any pending 
    // moves in the planner and serial buffers are all cleared and newly sent blocks will be 
    // locked out until a homing cycle or a kill lock command. Allows the user to disable the hard
    // limit setting if their limits are constantly triggering after a reset and move their axes.
    if (sys.state != STATE_ALARM) { 
      if (bit_isfalse(sys.execute,EXEC_ALARM)) {
        #ifndef LIMIT_SWITCHES_ACTIVE_HIGH
          if ((LIMIT_PIN & LIMIT_MASK) ^ LIMIT_MASK) {
        #else
          if (LIMIT_PIN & LIMIT_MASK) {
        #endif
          mc_reset(); // Initiate system kill.
          sys.execute |= EXEC_CRIT_EVENT; // Indicate hard limit critical event
        }
      }
    }  
  }
#else
  ISR(LIMIT_INT_vect)
  {
    // Ignore limit switches if already in an alarm state or in-process of executing an alarm.
    // When in the alarm state, Grbl should have been reset or will force a reset, so any pending 
    // moves in the planner and serial buffers are all cleared and newly sent blocks will be 
    // locked out until a homing cycle or a kill lock command. Allows the user to disable the hard
    // limit setting if their limits are constantly triggering after a reset and move their axes.
    if (sys.state != STATE_ALARM) { 
      if (bit_isfalse(sys.execute,EXEC_ALARM)) {
        mc_reset(); // Initiate system kill.
        sys.execute |= EXEC_CRIT_EVENT; // Indicate hard limit critical event
      }
    }  
  }
#endif


// Moves specified cycle axes all at homing rate, either approaching or disengaging the limit
// switches. Homing is a special motion case, where there is only an acceleration followed
// by abrupt asynchronous stops by each axes reaching their limit switch independently. The 
// asynchronous stops are handled by a system level axis lock mask, which prevents the stepper
// algorithm from executing step pulses.
// NOTE: Only the abort runtime command can interrupt this process.
void limits_go_home(uint8_t cycle_mask, bool approach, bool invert_pin, float homing_rate) 
{
  if (sys.execute & EXEC_RESET) { return; }
  uint8_t limit_state;
  #ifndef LIMIT_SWITCHES_ACTIVE_HIGH
    invert_pin = !invert_pin;
  #endif
  // Determine travel distance to the furthest homing switch based on user max travel settings.
  float max_travel = settings.max_travel[X_AXIS];
  if (max_travel < settings.max_travel[Y_AXIS]) { max_travel = settings.max_travel[Y_AXIS]; }
  if (max_travel < settings.max_travel[Z_AXIS]) { max_travel = settings.max_travel[Z_AXIS]; }
  max_travel *= 1.25; // Ensure homing switches engaged by over-estimating max travel.
  if (approach) { max_travel = -max_travel; }
  
  // Set target location and rate for active axes.
  float target[N_AXIS];
  uint8_t n_active_axis = 0;
  uint8_t i;
  for (i=0; i<N_AXIS; i++) {
    if (bit_istrue(cycle_mask,bit(i))) { 
      n_active_axis++;
      target[i] = max_travel; 
     } else {
      target[i] = 0.0;
    }
  }      
  if (bit_istrue(settings.homing_dir_mask,(1<<X_LIMIT_BIT))) { target[X_AXIS] = -target[X_AXIS]; }
  if (bit_istrue(settings.homing_dir_mask,(1<<Y_LIMIT_BIT))) { target[Y_AXIS] = -target[Y_AXIS]; }
  if (bit_istrue(settings.homing_dir_mask,(1<<Z_LIMIT_BIT))) { target[Z_AXIS] = -target[Z_AXIS]; }
  homing_rate *= sqrt(n_active_axis); // [sqrt(N_AXIS)] Adjust so individual axes all move at homing rate.

  // Setup homing axis locks based on cycle mask.
  uint8_t axislock = (STEPPING_MASK & ~STEP_MASK);
  if (bit_istrue(cycle_mask,bit(X_AXIS))) { axislock |= (1<<X_STEP_BIT); }
  if (bit_istrue(cycle_mask,bit(Y_AXIS))) { axislock |= (1<<Y_STEP_BIT); }
  if (bit_istrue(cycle_mask,bit(Z_AXIS))) { axislock |= (1<<Z_STEP_BIT); }
  sys.homing_axis_lock = axislock;
  
  // Perform homing cycle. Planner buffer should be empty, as required to initiate the homing cycle.
  plan_buffer_line(target, homing_rate, false); // Bypass mc_line(). Directly plan homing motion.
  st_prep_buffer(); // Prep first segment from newly planned block.
  st_wake_up(); // Initiate motion
  do {
    // Check limit state. Lock out cycle axes when they change.
    limit_state = LIMIT_PIN;
    if (invert_pin) { limit_state ^= LIMIT_MASK; }
//     if (axislock & (1<<X_STEP_BIT)) {
      if (limit_state & (1<<X_LIMIT_BIT)) { axislock &= ~(1<<X_STEP_BIT); }
//     }
//     if (axislock & (1<<Y_STEP_BIT)) {
      if (limit_state & (1<<Y_LIMIT_BIT)) { axislock &= ~(1<<Y_STEP_BIT); }
//     }
//     if (axislock & (1<<Z_STEP_BIT)) {
      if (limit_state & (1<<Z_LIMIT_BIT)) { axislock &= ~(1<<Z_STEP_BIT); }
//     }
    sys.homing_axis_lock = axislock;
    st_prep_buffer(); // Check and prep one segment. NOTE: Should take no longer than 200us.
    if (sys.execute & EXEC_RESET) { return; }
  } while (STEP_MASK & axislock);
  st_go_idle(); // Disable steppers. Axes motion should already be locked.
  plan_reset(); // Reset planner buffer. Ensure homing motion is cleared.
  st_reset(); // Reset step segment buffer. Ensure homing motion is cleared.
  delay_ms(settings.homing_debounce_delay); // Delay to allow transient dynamics to dissipate.
}


// Performs a soft limit check. Called from mc_line() only. Assumes the machine has been homed,
// and the workspace volume is in all negative space.
void limits_soft_check(float *target)
{
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) { 
    if (target[idx] > 0 || target[idx] < settings.max_travel[idx]) {  // NOTE: max_travel is stored as negative
    
      // Force feed hold if cycle is active. All buffered blocks are guaranteed to be within 
      // workspace volume so just come to a controlled stop so position is not lost. When complete
      // enter alarm mode.
      if (sys.state == STATE_CYCLE) {
        st_feed_hold();
        while (sys.state == STATE_HOLD) {
          protocol_execute_runtime();
          if (sys.abort) { return; }
        }
      }
      
      mc_reset(); // Issue system reset and ensure spindle and coolant are shutdown.
      sys.execute |= EXEC_CRIT_EVENT; // Indicate soft limit critical event
      protocol_execute_runtime(); // Execute to enter critical event loop and system abort
      return;
    
    }
  }
}
