/*
  stepper.h - stepper motor interface
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

#ifndef stepper_h
#define stepper_h 

#include <avr/io.h>
#include <avr/sleep.h>


#define StepModeT int8_t
#define SM_RUN 0
#define SM_HALT 1

extern StepModeT st_current_mode;
extern uint32_t iterations;            // The number of iterations left to complete the current_block

// Initialize and start the stepper motor subsystem
void st_init();

// Add a new linear movement to the buffer. steps_x, _y and _z is the signed, 
// relative motion in steps.
// pos_x, _y, _z is the position in absolute steps at the start of the move
// Microseconds specify how many microseconds the move should take to perform.
int st_buffer_block(int32_t steps_x, int32_t steps_y, int32_t steps_z, 
					 int32_t pos_x,   int32_t pos_y,   int32_t pos_z,
					 uint32_t rate, 
					 int16_t line_number);

// Returns true if the stepper buffer is full. This can allow the controlling
// program not to send more commands... Don't know if this is going to work.
char st_buffer_full();

// Set to true if the controller is running in manual mode:
extern char buttons_in_use;

// Block until all buffered steps are executed
//void st_synchronize();

// Cancel all pending steps
// void st_flush();

// Stop whatever is going on right now!
void st_stop();

// Execute the homing cycle
// void st_go_home();

// End current instruction and go to next one:
// void st_next();

// Continue stopped command:
// void st_continue();

#endif
