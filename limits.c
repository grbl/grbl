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
  #else // LIMIT_SWITCHES_ACTIVE_HIGH
    LIMIT_PORT &= ~(LIMIT_MASK); // Normal low operation. Requires external pull-down.
  #endif // !LIMIT_SWITCHES_ACTIVE_HIGH

  if (bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE)) {
    LIMIT_PCMSK |= LIMIT_MASK; // Enable specific pins of the Pin Change Interrupt
    PCICR |= (1 << LIMIT_INT); // Enable Pin Change Interrupt
  } else {
    LIMIT_PCMSK &= ~LIMIT_MASK; // Disable
    PCICR &= ~(1 << LIMIT_INT); 
  }
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


// Moves all specified axes in same specified direction (positive=true, negative=false)
// and at the homing rate. Homing is a special motion case, where there is only an 
// acceleration followed by abrupt asynchronous stops by each axes reaching their limit 
// switch independently. Instead of shoehorning homing cycles into the main stepper 
// algorithm and overcomplicate things, a stripped-down, lite version of the stepper 
// algorithm is written here. This also lets users hack and tune this code freely for
// their own particular needs without affecting the rest of Grbl.
// NOTE: Only the abort runtime command can interrupt this process.
static void homing_cycle(uint8_t cycle_mask, bool pos_dir, bool invert_pin, float homing_rate) 
{
  if (sys.execute & EXEC_RESET) { return; }
  uint8_t limit_state;
  #ifndef LIMIT_SWITCHES_ACTIVE_HIGH
    invert_pin = !invert_pin;
  #endif

  // Compute target location for homing all axes. Homing axis lock will freeze non-cycle axes.
  float target[N_AXIS];
  target[X_AXIS] = settings.max_travel[X_AXIS];
  if (target[X_AXIS] < settings.max_travel[Y_AXIS]) { target[X_AXIS] = settings.max_travel[Y_AXIS]; }
  if (target[X_AXIS] < settings.max_travel[Z_AXIS]) { target[X_AXIS] = settings.max_travel[Z_AXIS]; }
  target[X_AXIS] *= 2.0;
  if (pos_dir) { target[X_AXIS] = -target[X_AXIS]; }
  target[Y_AXIS] = target[X_AXIS];
  target[Z_AXIS] = target[X_AXIS];
  homing_rate *= 1.7320; // [sqrt(N_AXIS)] Adjust so individual axes all move at homing rate.

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
  while (STEP_MASK & axislock) {
    // Check limit state.
    limit_state = LIMIT_PIN;
    if (invert_pin) { limit_state ^= LIMIT_MASK; }
    if (axislock & (1<<X_STEP_BIT)) {
      if (limit_state & (1<<X_LIMIT_BIT)) { axislock &= ~(1<<X_STEP_BIT); }
    }
    if (axislock & (1<<Y_STEP_BIT)) {
      if (limit_state & (1<<Y_LIMIT_BIT)) { axislock &= ~(1<<Y_STEP_BIT); }
    }
    if (axislock & (1<<Z_STEP_BIT)) {
      if (limit_state & (1<<Z_LIMIT_BIT)) { axislock &= ~(1<<Z_STEP_BIT); }
    }
    sys.homing_axis_lock = axislock;
    st_prep_buffer(); // Check and prep one segment. NOTE: Should take no longer than 200us.
    if (sys.execute & EXEC_RESET) { return; }
  }
  st_go_idle(); // Disable steppers. Axes motion should already be locked.
  plan_init(); // Reset planner buffer. Ensure homing motion is cleared.
  st_reset(); // Reset step segment buffer. Ensure homing motion is cleared.
  delay_ms(settings.homing_debounce_delay);
}


void limits_go_home() 
{  
  plan_init(); // Reset planner buffer before beginning homing cycles.
  
  // Search to engage all axes limit switches at faster homing seek rate.
  homing_cycle(HOMING_SEARCH_CYCLE_0, true, false, settings.homing_seek_rate);  // Search cycle 0
  #ifdef HOMING_SEARCH_CYCLE_1
    homing_cycle(HOMING_SEARCH_CYCLE_1, true, false, settings.homing_seek_rate);  // Search cycle 1
  #endif
  #ifdef HOMING_SEARCH_CYCLE_2
    homing_cycle(HOMING_SEARCH_CYCLE_2, true, false, settings.homing_seek_rate);  // Search cycle 2
  #endif
    
  // Now in proximity of all limits. Carefully leave and approach switches in multiple cycles
  // to precisely hone in on the machine zero location. Moves at slower homing feed rate.
  int8_t n_cycle = N_HOMING_LOCATE_CYCLE;
  while (n_cycle--) {
    // Leave all switches to release them. After cycles complete, this is machine zero.
    homing_cycle(HOMING_LOCATE_CYCLE, false, true, settings.homing_feed_rate);

    if (n_cycle > 0) {
      // Re-approach all switches to re-engage them.
      homing_cycle(HOMING_LOCATE_CYCLE, true, false, settings.homing_feed_rate);
    }
  }
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
