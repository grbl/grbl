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

#define STEPPER_MODE_STOPPED 0
#define STEPPER_MODE_RUNNING 1
#define STEPPER_MODE_LIMIT_OVERRUN 2
#define STEPPER_MODE_HOMING 3

// Initialize and start the stepper motor subsystem
void st_init();

// Returns a bitmask with the stepper bit for the given axis set
uint8_t st_bit_for_stepper(int axis);

// Buffer a pace change. Pace is the rate with which steps are executed. It is measured in microseconds from step to step. 
// It is continually adjusted to achieve constant actual feed rate. Unless pace-changes was buffered along with the steps 
// they govern they might change at slightly wrong moments in time as the pace would change while the stepper buffer was
// still churning out the previous movement.
void st_buffer_pace(uint32_t microseconds);

// Buffer a new instruction for the steppers
inline void st_buffer_step(uint8_t motor_port_bits);

// Block until all buffered steps are executed
void st_synchronize();

// Cancel all pending steps
void st_flush();

// Start the stepper subsystem
void st_start();

// Execute all buffered steps, then stop the stepper subsystem
inline void st_stop();

// Execute the homing cycle
void st_go_home();

// Echo steps to serial port? (true/false)
void st_set_echo(int value);

// Convert from millimeters to step-counts along the designated axis
int32_t st_millimeters_to_steps(double millimeters, int axis);

#endif
