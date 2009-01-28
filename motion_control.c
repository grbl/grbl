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
#include "stepper.h"

// position represents the current position of the head measured in steps
// target is the target for the current linear motion
// step_count contains the absolute values of the steps to travel along each axis
// direction is the sign of the motion for each axis (-1: reverse, 0: standby, 1: forward)

#define MODE_AT_REST 0
#define MODE_LINEAR 1
#define MODE_ARC 2
#define MODE_DWELL 3
#define MODE_HOME 4

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

struct ArcMotionParameters {
  int8_t angular_direction; // 1 = clockwise, -1 = anticlockwise
  uint32_t circle_x, circle_y, target_x, target_y; // current position and target position in the 
                                                   // local coordinate system of the circle where [0,0] is the 
                                                   // center of the circle.
  int32_t error, x2, y2;                           // error is always == (circle_x**2 + circle_y**2 - radius**2), 
                                                   // x2 is always 2*x, y2 is always 2*y
  uint8_t axis_x, axis_y;                           // maps the circle axes to stepper axes
  int32_t target[3];                                // The target position in absolute steps
};

/* The whole state of the motion-control-system in one struct. Makes the code a little bit hard to 
   read, but lets us initialize the state of the system by just clearing a single, contigous block of memory.   
   By overlaying the variables of the different modes in a union we save a few bytes of precious SRAM.
*/
struct MotionControlState {
  int8_t mode;            // The current operation mode
  int32_t position[3];    // The current position of the tool in absolute steps
  int32_t pace;  // Microseconds between each update in the current mode
  union {
    struct LinearMotionParameters linear; // variables used in MODE_LINEAR
    struct ArcMotionParameters arc;       // variables used in MODE_ARC
    uint32_t dwell_milliseconds;          // variable used in MODE_DWELL
  };
};
struct MotionControlState state;

uint8_t direction_bits; // The direction bits to be used with any upcoming step-instruction

void enable_steppers();
void disable_steppers();
void set_direction_bits(int8_t *direction);
inline void step_steppers(uint8_t *enabled);
inline void step_axis(uint8_t axis);

void mc_init()
{
	// Initialize state variables
  memset(&state, 0, sizeof(state));
}

void mc_dwell(uint32_t milliseconds) 
{
  st_synchronize();
  state.mode = MODE_DWELL;
  state.dwell_milliseconds = milliseconds;
  state.pace = 1000;
}

void mc_linear_motion(double x, double y, double z, float feed_rate, int invert_feed_rate)
{
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
  set_direction_bits(state.linear.direction);

  // Calculate the microseconds we need to wait between each step to achieve the desired feed rate
  if (invert_feed_rate) {
  	state.pace = 
  	  (feed_rate*1000000)/state.linear.maximum_steps;	  
  } else {
  	// Ask old Phytagoras how many millimeters our next move is going to take us:
  	float millimeters_of_travel = 
      sqrt(pow((X_STEPS_PER_MM*state.linear.step_count[X_AXIS]),2) + 
           pow((Y_STEPS_PER_MM*state.linear.step_count[Y_AXIS]),2) + 
           pow((Z_STEPS_PER_MM*state.linear.step_count[Z_AXIS]),2));
  	state.pace = 
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

	} else {
    state.mode = MODE_AT_REST;
	}
}

void mc_arc(double theta, double angular_travel, double radius, uint32_t *target)
{
  state.mode = MODE_ARC;
  // Calculate the initial position and target position in the local coordinate system of the circle
  state.arc.circle_x = round(sin(theta)*radius); 
  state.arc.circle_y = round(cos(theta)*radius);
  state.arc.target_x = trunc(sin(theta+angular_travel)*(radius-0.5));
  state.arc.target_y = trunc(cos(theta+angular_travel)*(radius-0.5));
  // Determine angular direction (+1 = clockwise, -1 = counterclockwise)
  state.arc.angular_direction = sign(angular_travel);
  // The "error" factor is kept up to date so that it is always == (x**2+y**2-radius**2). When error 
  // <0 we are inside the circle, when it is >0 we are outside of the circle, and when it is 0 we 
  // are exactly on top of the circle.
  state.arc.error = round(pow(state.arc.circle_x,2) + pow(state.arc.circle_y,2) - pow(radius,2));
  // Because the error-value moves in steps of (+/-)2x+1 and (+/-)2y+1 we save a couple of multiplications
  // by keeping track of the doubles of the circle coordinates at all times.
  state.arc.x2 = 2*state.arc.circle_x;
  state.arc.y2 = 2*state.arc.circle_y; 
}

void step_arc_along_x(dx,dy) 
{
  uint32_t diagonal_error;
  state.arc.circle_x+=dx;
  state.arc.error += 1+state.arc.x2*dx;
  state.arc.x2 += 2*dx;
  diagonal_error = state.arc.error + 1 + state.arc.y2*dy;
  if(abs(state.arc.error) < abs(diagonal_error)) {
    state.arc.circle_y += dy;
    state.arc.y2 += 2*dy;
    state.arc.error = diagonal_error;
  };
}

void step_arc_along_y(dx,dy) 
{  
  uint32_t diagonal_error;
  state.arc.circle_y+=dy; 
  state.arc.error += 1+state.arc.y2*dy; 
  state.arc.y2 += 2*dy; 
  diagonal_error = state.arc.error + 1 + state.arc.x2*dx; 
  if(abs(state.arc.error) < abs(diagonal_error)) { 
    state.arc.circle_x += dx; 
    state.arc.x2 += 2*dx; 
    state.arc.error = diagonal_error; 
  }
}

/*
 Quandrants of the circle
       \ 7|0 /
        \ | / 
      6  \|/  1    y+
 ---------|-----------
      5  /|\  2    y-
        / | \  
   x-  / 4|3 \ x+           */

int quadrant(uint32_t x,uint32_t y)
{
  // determine if the coordinate is in the quadrants 0,3,4 or 7
  register int quad0347 = abs(x)<abs(y);
  
  if (x<0) { // quad 4567
    if (y<0) { // quad 45
      return(quad0347 ? 4 : 5);
    } else { // quad 67
      return(quad0347 ? 7 : 6);
    }
  } else {
    if (y<0) { // quad 23
      return(quad0347 ? 3 : 2);
    } else { // quad 01
      return(quad0347 ? 0 : 1);
    }
  }  
}

void perform_arc()
{
  int q = quadrant(state.arc.circle_x, state.arc.circle_y);
  
  if (state.arc.angular_direction) {
    switch (q) {
      case 0: while(state.arc.circle_x>state.arc.circle_y) { step_arc_along_x(1,-1); }
      case 1: while(state.arc.circle_y>0) { step_arc_along_y(1,-1); }
      case 2: while(state.arc.circle_y>-state.arc.circle_x) { step_arc_along_y(-1,-1); }
      case 3: while(state.arc.circle_x>0) { step_arc_along_x(-1,-1); }
      case 4: while(state.arc.circle_y<state.arc.circle_x) { step_arc_along_x(-1,1); }
      case 5: while(state.arc.circle_y<0) { step_arc_along_y(-1,1); }
      case 6: while(state.arc.circle_y<-state.arc.circle_x) { step_arc_along_y(1,1); }
      case 7: while(state.arc.circle_x<0) { step_arc_along_x(1,1); }
    }
  } else {
    switch (q) {
      case 7: while(state.arc.circle_y>-state.arc.circle_x) { step_arc_along_x(-1,-1); }
      case 6: while(state.arc.circle_y>0)  { step_arc_along_y(-1,-1); }
      case 5: while(state.arc.circle_y>state.arc.circle_x)  { step_arc_along_y(1,-1); }
      case 4: while(state.arc.circle_x<0)  { step_arc_along_x(1,-1); }
      case 3: while(state.arc.circle_y<-state.arc.circle_x) { step_arc_along_x(1,1); }      
      case 2: while(state.arc.circle_y<0)  { step_arc_along_y(1,1); }      
      case 1: while(state.arc.circle_y<state.arc.circle_x)  { step_arc_along_y(-1,1); }
      case 0: while(state.arc.circle_x>0)  { step_arc_along_x(-1,1); }
    }    
  }
}

void mc_go_home()
{
  state.mode = MODE_HOME;
}

void perform_go_home() 
{
  st_go_home();
  clear_vector(state.position); // By definition this is location [0, 0, 0]
  state.mode = MODE_AT_REST;
}

void mc_execute() {
  st_set_pace(state.pace);
  while(state.mode) {
    switch(state.mode) {
      case MODE_AT_REST: break;
      case MODE_DWELL: _delay_ms(state.dwell_milliseconds); state.mode = MODE_AT_REST; break;
      case MODE_LINEAR: perform_linear_motion();
      case MODE_HOME: perform_go_home();
    }
  }
}

int mc_status() 
{
  return(state.mode);
}


// Set the direction pins for the stepper motors according to the provided vector. 
// direction is an array of three 8 bit integers representing the direction of 
// each motor. The values should be -1 (reverse), 0 or 1 (forward).
void set_direction_bits(int8_t *direction) 
{
  /* Sorry about this convoluted code! It uses the fact that bit 7 of each direction
     int is set when the direction == -1, but is 0 when direction is forward. This
     way we can generate the whole direction bit-mask without doing any comparisions
     or branching. Fast and compact, yet practically unreadable. Sorry sorry sorry.
  */
  direction_bits = ~(
    ((direction[X_AXIS]&128)>>(7-X_DIRECTION_BIT)) |
    ((direction[Y_AXIS]&128)>>(7-Y_DIRECTION_BIT)) |
    ((direction[Z_AXIS]&128)>>(7-Z_DIRECTION_BIT))
  );
}

// Step enabled steppers. Enabled should be an array of three bytes. Each byte represent one
// stepper motor in the order X, Y, Z. Set the bytes of the steppers you want to step to
// 1, and the rest to 0. 
inline void step_steppers(uint8_t *enabled) 
{
  st_buffer_step(direction_bits | enabled[X_AXIS]<<X_STEP_BIT | enabled[Y_AXIS]<<Y_STEP_BIT | enabled[Z_AXIS]<<Z_STEP_BIT);
}

// Step only one motor
inline void step_axis(uint8_t axis) 
{
  switch (axis) {
    case X_AXIS: st_buffer_step(direction_bits | (1<<X_STEP_BIT)); break;
    case Y_AXIS: st_buffer_step(direction_bits | (1<<Y_STEP_BIT)); break;
    case Z_AXIS: st_buffer_step(direction_bits | (1<<Z_STEP_BIT)); break;
  }
}
