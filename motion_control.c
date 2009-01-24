/*
  motion_control.c - cartesian robot controller.
  Part of Grbl

  Copyright (c) 2009 Simen Svale Skogsrud

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

/* This code was inspired by the Arduino GCode_Interpreter by Mike Ellery. */

#include <avr/io.h>
#include "config.h"
#include "motion_control.h"
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#include "nuts_bolts.h"

// position represents the current position of the head measured in steps
// target is the target for the current linear motion
// step_count contains the absolute values of the steps to travel along each axis
// direction is the sign of the motion for each axis (-1: reverse, 0: standby, 1: forward)

#define MODE_AT_REST 0
#define MODE_LINEAR 1
#define MODE_ARC 2
#define MODE_DWELL 3
#define MODE_HOME 4
#define MODE_LIMIT_OVERRUN -1

#define PHASE_HOME_RETURN 0
#define PHASE_HOME_NUDGE 1

#define ONE_MINUTE_OF_MICROSECONDS 60000000

// Parameters when mode is MODE_ARC
struct LinearMotionParameters {
  int8_t direction[3];  // The direction of travel along each axis (-1, 0 or 1)
  uint16_t feed_rate;
  int32_t target[3], // The target position in absolute steps
    step_count[3],   // Absolute steps of travel along each axis
    counter[3],      // A counter used in the bresenham algorithm for line plotting
    maximum_steps;   // The larges absolute step-count of any axis
};

// Parameters when mode is MODE_LINEAR
struct ArcMotionParameters {
  uint32_t radius;
  int16_t degrees;
  int ccw;
};

struct HomeCycleParameters {
  int8_t direction[3];  // The direction of travel along each axis (-1, 0 or 1)
  int8_t phase;       // current phase of the home cycle. 
  int8_t away[3];     // a vector of booleans. True for each axis that is still away.
};

/* The whole state of the motion-control-system in one struct. Makes the code a little bit hard to 
   read, but lets us initialize the state of the system by just clearing a single, contigous block of memory.   
   By overlaying the variables of the different modes in a union we save a few bytes of precious SRAM.
*/
struct MotionControlState {
  int8_t mode;            // The current operation mode
  int32_t position[3];    // The current position of the tool in absolute steps
  int32_t update_delay_us;  // Microseconds between each update in the current mode
  union {
    struct LinearMotionParameters linear; // variables used in MODE_LINEAR
    struct ArcMotionParameters arc;       // variables used in MODE_ARC
    struct HomeCycleParameters home;      // variables used in MODE_HOME
    uint32_t dwell_milliseconds;          // variable used in MODE_DWELL
    int8_t limit_overrun_direction[3];    // variable used in MODE_LIMIT_OVERRUN
  };
};
struct MotionControlState state;

int check_limit_switches();
void enable_steppers();
void disable_steppers();
void set_direction_pins(int8_t *direction);
inline void step_steppers(uint8_t *enabled);
void limit_overrun(uint8_t *direction);
int check_limit_switch(int axis);
inline void step_axis(uint8_t axis);

void mc_init()
{
	// Initialize state variables
  memset(&state, 0, sizeof(state));

	// Configure directions of interface pins
  STEP_DDR   |= STEP_MASK;
  DIRECTION_DDR |= DIRECTION_MASK;
  LIMIT_DDR &= ~(LIMIT_MASK);
  STEPPERS_ENABLE_DDR |= 1<<STEPPERS_ENABLE_BIT;
	
	disable_steppers();
}

void limit_overrun(uint8_t *direction) 
{
  state.mode = MODE_LIMIT_OVERRUN;
  memcpy(state.limit_overrun_direction, direction, sizeof(state.limit_overrun_direction));
}

void mc_dwell(uint32_t milliseconds) 
{
  mc_wait();
  state.mode = MODE_DWELL;
  state.dwell_milliseconds = milliseconds;
  state.update_delay_us = 1000;
}

void mc_linear_motion(double x, double y, double z, float feed_rate, int invert_feed_rate)
{
  mc_wait();
  state.mode = MODE_LINEAR;
  
	state.linear.target[X_AXIS] = trunc(x*X_STEPS_PER_MM);
	state.linear.target[Y_AXIS] = trunc(y*Y_STEPS_PER_MM);
	state.linear.target[Z_AXIS] = trunc(z*Z_STEPS_PER_MM);

  uint8_t axis; // loop variable

  // Determine direction and travel magnitude for each axis
  for(axis = X_AXIS; axis <= Z_AXIS; axis++) {
  	state.linear.step_count[axis] = abs(state.linear.target[axis] - state.position[axis]);
    state.linear.direction[axis] = sign(state.linear.step_count[axis]);
  }
  // Find the magnitude of the axis with the longest travel
	state.linear.maximum_steps = max(state.linear.step_count[Z_AXIS], 
	  max(state.linear.step_count[X_AXIS], state.linear.step_count[Y_AXIS]));

	// Set up a neat counter for each axis
  for(axis = X_AXIS; axis <= Z_AXIS; axis++) {
    state.linear.counter[axis] = -state.linear.maximum_steps/2;
  }

	// Set our direction pins 
  set_direction_pins(state.linear.direction);

  // Calculate the microseconds we need to wait between each step to achieve the desired feed rate
  if (invert_feed_rate) {
  	state.update_delay_us = 
  	  (feed_rate*1000000.0)/state.linear.maximum_steps;	  
  } else {
  	// Ask old Phytagoras how many millimeters our next move is going to take us:
  	float millimeters_of_travel = 
      sqrt(pow((X_STEPS_PER_MM*state.linear.step_count[X_AXIS]),2) + 
           pow((Y_STEPS_PER_MM*state.linear.step_count[Y_AXIS]),2) + 
           pow((Z_STEPS_PER_MM*state.linear.step_count[Z_AXIS]),2));
  	state.update_delay_us = 
  	  ((millimeters_of_travel * ONE_MINUTE_OF_MICROSECONDS) / feed_rate) / state.linear.maximum_steps;	  
  }
}

void perform_linear_motion()
{
	// Flags to keep track of which axes to step
  uint8_t step[3];
  uint8_t axis; // loop variable
  
	// Trace the line
  clear_vector(step);
  for(axis = X_AXIS; axis <= Z_AXIS; axis++) {
		if (state.linear.target[axis] != state.position[axis])
		{
			state.linear.counter[axis] += state.linear.step_count[axis];
			if (state.linear.counter[axis] > 0)
			{
        step[axis] = true;
				state.linear.counter[axis] -= state.linear.maximum_steps;
        state.position[axis] += state.linear.direction[axis];
			}
		}
	}

	if (step[X_AXIS] | step[Y_AXIS] | step[Z_AXIS]) {
    step_steppers(step);

		// If we trip any limit switch while moving: Abort, abort!
		if (check_limit_switches()) { 
      limit_overrun(state.linear.direction);
		}

		_delay_us(state.update_delay_us);
	} else {
    state.mode = MODE_AT_REST;
	}
}

void mc_go_home()
{
  state.mode = MODE_HOME;
  memset(state.home.direction, -1, sizeof(state.home.direction)); // direction = [-1,-1,-1]
  set_direction_pins(state.home.direction);
  clear_vector(state.home.away);
}

void perform_go_home() 
{
  int axis;
  if(state.home.phase == PHASE_HOME_RETURN) {
    // We are running all axes in reverse until all limit switches are tripped
    // Check all limit switches:
    for(axis=X_AXIS; axis <= Z_AXIS; axis++) {
      state.home.away[axis] |= check_limit_switch(axis);
    }
    // Step steppers. First retract along Z-axis. Then X and Y.
    if(state.home.away[Z_AXIS]) {
      step_axis(Z_AXIS);
    } else {
      // Check if all axes are home
      if(!(state.home.away[X_AXIS] || state.home.away[Y_AXIS])) {
        // All axes are home, prepare next phase: to nudge the tool carefully out of the limit switches
        memset(state.home.direction, 1, sizeof(state.home.direction)); // direction = [1,1,1]
        set_direction_pins(state.home.direction);
        state.home.phase == PHASE_HOME_NUDGE;
        return;
      }
      step_steppers(state.home.away);
    }
  } else {    
    for(axis=X_AXIS; axis <= Z_AXIS; axis++) {
      if(check_limit_switch(axis)) {
        step_axis(axis);
        return;
      }
    }
    // When this code is reached it means all axes are free of their limit-switches. Complete the cycle and rest:
    clear_vector(state.position); // By definition this is location [0, 0, 0]
    state.mode = MODE_AT_REST;
  }
}

void mc_execute() {
  enable_steppers();
  while(state.mode) {
    switch(state.mode) {
      case MODE_AT_REST: break;
      case MODE_DWELL: _delay_ms(state.dwell_milliseconds); state.mode = MODE_AT_REST; break;
      case MODE_LINEAR: perform_linear_motion();
      case MODE_HOME: perform_go_home();
    }
    _delay_us(state.update_delay_us);
  }
  disable_steppers();
}

void mc_wait() {  
  return; // No concurrency support yet. So waiting for all to pass is moot.
}

int mc_status() 
{
  return(state.mode);
}

int check_limit_switches()
{
  // Dual read as crude debounce
  return((LIMIT_PORT & LIMIT_MASK) | (LIMIT_PORT & LIMIT_MASK));
}

int check_limit_switch(int axis)
{
  uint8_t mask = 0;
  switch (axis) {
    case X_AXIS: mask = 1<<X_LIMIT_BIT; break;
    case Y_AXIS: mask = 1<<Y_LIMIT_BIT; break;
    case Z_AXIS: mask = 1<<Z_LIMIT_BIT; break;
  }
  return((LIMIT_PORT&mask) || (LIMIT_PORT&mask));    
}

void enable_steppers()
{
  STEPPERS_ENABLE_PORT |= 1<<STEPPERS_ENABLE_BIT;
}

void disable_steppers()
{
  STEPPERS_ENABLE_PORT &= ~(1<<STEPPERS_ENABLE_BIT);
}

// Set the direction pins for the stepper motors according to the provided vector. 
// direction is an array of three 8 bit integers representing the direction of 
// each motor. The values should be -1 (reverse), 0 or 1 (forward).
void set_direction_pins(int8_t *direction) 
{
  /* Sorry about this convoluted code! It uses the fact that bit 7 of each direction
     int is set when the direction == -1, but is 0 when direction is forward. This
     way we can generate the whole direction bit-mask without doing any comparisions
     or branching. Fast and compact, yet practically unreadable. Sorry sorry sorry.
  */
  uint8_t forward_bits = ~(
    ((direction[X_AXIS]&128)>>(7-X_DIRECTION_BIT)) |
    ((direction[Y_AXIS]&128)>>(7-Y_DIRECTION_BIT)) |
    ((direction[Z_AXIS]&128)>>(7-Z_DIRECTION_BIT))
  );
  DIRECTION_PORT = DIRECTION_PORT & ~(DIRECTION_MASK) | forward_bits;
}

// Step enabled steppers. Enabled should be an array of three bytes. Each byte represent one
// stepper motor in the order X, Y, Z. Set the bytes of the steppers you want to step to
// 1, and the rest to 0. 
inline void step_steppers(uint8_t *enabled) 
{
  STEP_PORT |= enabled[X_AXIS]<<X_STEP_BIT | enabled[Y_AXIS]<<Y_STEP_BIT | enabled[Z_AXIS]<<Z_STEP_BIT;
  _delay_us(5);
  STEP_PORT &= ~STEP_MASK;
}

// Step only one motor
inline void step_axis(uint8_t axis) 
{
  uint8_t mask = 0;
  switch (axis) {
    case X_AXIS: mask = 1<<X_STEP_BIT; break;
    case Y_AXIS: mask = 1<<Y_STEP_BIT; break;
    case Z_AXIS: mask = 1<<Z_STEP_BIT; break;
  }
  STEP_PORT &= mask;
  _delay_us(5);
  STEP_PORT &= ~STEP_MASK;  
}
