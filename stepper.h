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

// Set the rate steps are taken from the buffer and executed
void st_set_pace(uint32_t microseconds);

// Buffer a new instruction for the steppers
void st_buffer_step(uint8_t motor_port_bits);

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

#endif
