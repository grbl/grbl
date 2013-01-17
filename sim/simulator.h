/*
  simulator.h - functions to simulate how the buffer is emptied and the
    stepper interrupt is called

  Part of Grbl Simulator

  Copyright (c) 2012 Jens Geisler

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

#ifndef simulator_h
#define simulator_h

#include <stdio.h>
#include "../nuts_bolts.h"

// Output file handles
extern FILE *block_out_file;
extern FILE *step_out_file;

// This variable is needed to determine if execute_runtime() is called in a loop
// waiting for the buffer to empty, as in plan_synchronize()
extern int runtime_second_call;

// Minimum time step for printing stepper values. Given by user via command line
extern double step_time;

// global system variable structure for position etc.
extern system_t sys;
extern uint8_t print_comment;


// Call the stepper interrupt until one block is finished
void sim_stepper();

// Check if buffer is full or if plan_synchronize() wants to clear the buffer
void handle_buffer();

// Print information about the most recently inserted block
void printBlock();

// Calculate the time between stepper interrupt calls from TCCR1B and OCR1A AVR registers
// which are set in config_step_timer in stepper.c
// This reconstructs the stepper-internal value of variable st.cycles_per_step_event
// The reconstruction is done to truely decouple the simulator from the actual grbl code
double get_step_time();

#endif
