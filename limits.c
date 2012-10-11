/*
  limits.c - code pertaining to limit-switches and performing the homing cycle
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2012 Sungeun K. Jeon

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

#define MICROSECONDS_PER_ACCELERATION_TICK  (1000000/ACCELERATION_TICKS_PER_SECOND)

void limits_init() 
{
  LIMIT_DDR &= ~(LIMIT_MASK); // Set as input pins
  LIMIT_PORT |= (LIMIT_MASK); // Enable internal pull-up resistors. Normal high operation.
}

// Moves all specified axes in same specified direction (positive=true, negative=false)
// and at the homing rate. Homing is a special motion case, where there is only an 
// acceleration followed by abrupt asynchronous stops by each axes reaching their limit 
// switch independently. Instead of shoehorning homing cycles into the main stepper 
// algorithm and overcomplicate things, a stripped-down, lite version of the stepper 
// algorithm is written here. This also lets users hack and tune this code freely for
// their own particular needs without affecting the rest of Grbl.
// NOTE: Only the abort runtime command can interrupt this process.
static void homing_cycle(bool x_axis, bool y_axis, bool z_axis, int8_t pos_dir, 
                         bool invert_pin, float homing_rate) 
{
  // Determine governing axes with finest step resolution per distance for the Bresenham
  // algorithm. This solves the issue when homing multiple axes that have different 
  // resolutions without exceeding system acceleration setting. It doesn't have to be
  // perfect since homing locates machine zero, but should create for a more consistent 
  // and speedy homing routine.
  // NOTE: For each axes enabled, the following calculations assume they physically move 
  // an equal distance over each time step until they hit a limit switch, aka dogleg.
  uint32_t steps[3];
  clear_vector(steps);
  if (x_axis) { steps[X_AXIS] = lround(settings.steps_per_mm[X_AXIS]); }
  if (y_axis) { steps[Y_AXIS] = lround(settings.steps_per_mm[Y_AXIS]); }
  if (z_axis) { steps[Z_AXIS] = lround(settings.steps_per_mm[Z_AXIS]); }
  uint32_t step_event_count = max(steps[X_AXIS], max(steps[Y_AXIS], steps[Z_AXIS]));  
  
  // To ensure global acceleration is not exceeded, reduce the governing axes nominal rate
  // by adjusting the actual axes distance traveled per step. This is the same procedure
  // used in the main planner to account for distance traveled when moving multiple axes.
  // NOTE: When axis acceleration independence is installed, this will be updated to move
  // all axes at their maximum acceleration and rate.
  float ds = step_event_count/sqrt(x_axis+y_axis+z_axis);

  // Compute the adjusted step rate change with each acceleration tick. (in step/min/acceleration_tick)
  uint32_t delta_rate = ceil( ds*settings.acceleration/(60*ACCELERATION_TICKS_PER_SECOND));
  
  // Nominal and initial time increment per step. Nominal should always be greater then 3
  // usec, since they are based on the same parameters as the main stepper routine. Initial
  // is based on the MINIMUM_STEPS_PER_MINUTE config.
  uint32_t dt_min = lround(1000000*60/(ds*homing_rate)); // Cruising (usec/step)
  uint32_t dt = 1000000*60/MINIMUM_STEPS_PER_MINUTE; // Initial (usec/step)
      
  // Set default out_bits. 
  uint8_t out_bits0 = settings.invert_mask;
  if (!pos_dir) { out_bits0 ^= DIRECTION_MASK; }   // Invert bits, if negative dir.
  
  // Initialize stepping variables
  int32_t counter_x = -(step_event_count >> 1); // Bresenham counters
  int32_t counter_y = counter_x;
  int32_t counter_z = counter_x;
  uint32_t step_delay = dt-settings.pulse_microseconds;  // Step delay after pulse
  uint32_t step_rate = 0;  // Tracks step rate. Initialized from 0 rate. (in step/min)
  uint32_t trap_counter = MICROSECONDS_PER_ACCELERATION_TICK/2; // Acceleration trapezoid counter
  uint8_t out_bits;
  uint8_t limit_state;
  for(;;) {
  
    // Reset out bits. Both direction and step pins appropriately inverted and set.
    out_bits = out_bits0;
    
    // Get limit pin state.
    limit_state = LIMIT_PIN;
    if (invert_pin) { limit_state ^= LIMIT_MASK; } // If leaving switch, invert to move.
    
    // Set step pins by Bresenham line algorithm. If limit switch reached, disable and
    // flag for completion.
    if (x_axis) {
      counter_x += steps[X_AXIS];
      if (counter_x > 0) {
        if (limit_state & (1<<X_LIMIT_BIT)) { out_bits ^= (1<<X_STEP_BIT); }
        else { x_axis = false; }
        counter_x -= step_event_count;
      }
    }
    if (y_axis) {
      counter_y += steps[Y_AXIS];
      if (counter_y > 0) {
        if (limit_state & (1<<Y_LIMIT_BIT)) { out_bits ^= (1<<Y_STEP_BIT); }
        else { y_axis = false; }
        counter_y -= step_event_count;
      }
    }
    if (z_axis) {
      counter_z += steps[Z_AXIS];
      if (counter_z > 0) {
        if (limit_state & (1<<Z_LIMIT_BIT)) { out_bits ^= (1<<Z_STEP_BIT); }
        else { z_axis = false; }
        counter_z -= step_event_count;
      }
    }        
    
    // Check if we are done or for system abort
    protocol_execute_runtime();
    if (!(x_axis || y_axis || z_axis) || sys.abort) { return; }
        
    // Perform step.
    STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | (out_bits & STEP_MASK);
    delay_us(settings.pulse_microseconds);
    STEPPING_PORT = out_bits0;
    delay_us(step_delay);
    
    // Track and set the next step delay, if required. This routine uses another Bresenham
    // line algorithm to follow the constant acceleration line in the velocity and time 
    // domain. This is a lite version of the same routine used in the main stepper program.
    if (dt > dt_min) { // Unless cruising, check for time update.
      trap_counter += dt; // Track time passed since last update.
      if (trap_counter > MICROSECONDS_PER_ACCELERATION_TICK) {
        trap_counter -= MICROSECONDS_PER_ACCELERATION_TICK;
        step_rate += delta_rate; // Increment velocity
        dt = (1000000*60)/step_rate; // Compute new time increment
        if (dt < dt_min) {dt = dt_min;}  // If target rate reached, cruise.
        step_delay = dt-settings.pulse_microseconds;
      }
    }
  }
}


void limits_go_home() 
{
  plan_synchronize();  // Empty all motions in buffer.
  
  // Jog all axes toward home to engage their limit switches at faster homing seek rate.
  homing_cycle(false, false, true, true, false, settings.homing_seek_rate); // First jog the z axis
  homing_cycle(true, true, false, true, false, settings.homing_seek_rate);   // Then jog the x and y axis
  delay_ms(settings.homing_debounce_delay); // Delay to debounce signal
    
  // Now in proximity of all limits. Carefully leave and approach switches in multiple cycles
  // to precisely hone in on the machine zero location. Moves at slower homing feed rate.
  int8_t n_cycle = N_HOMING_CYCLE;
  while (n_cycle--) {
    // Leave all switches to release them. After cycles complete, this is machine zero.
    homing_cycle(true, true, true, false, true, settings.homing_feed_rate);
    delay_ms(settings.homing_debounce_delay);
    
    if (n_cycle > 0) {
      // Re-approach all switches to re-engage them.
      homing_cycle(true, true, true, true, false, settings.homing_feed_rate);
      delay_ms(settings.homing_debounce_delay);
    }
  }
}
