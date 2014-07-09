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
#include "../system.h"
#include "platform.h"

//simulation globals
typedef struct sim_vars {
  uint64_t masterclock;
  double sim_time;  //current time of the simulation
  uint8_t started;  //don't start timers until first char recieved.
  uint8_t exit; 
  float speedup;
  int32_t baud_ticks;
  double next_print_time;

} sim_vars_t;  
extern sim_vars_t sim;


typedef struct arg_vars {
  // Output file handles
  FILE *block_out_file;
  FILE *step_out_file;
  FILE *grbl_out_file;
  // Minimum time step for printing stepper values.  //Given by user via command line
  double step_time;
  //char to prefix comments; default  '#' 
  uint8_t comment_char;   
  
} arg_vars_t;
extern arg_vars_t args;


// global system variable structure for position etc.
extern system_t sys;

// setup avr simulation
void init_simulator(float time_multiplier);

//shutdown simulator - close open files
int shutdown_simulator(uint8_t exitflag);

//simulates the hardware until sim.exit is set.
void sim_loop();

// Call the stepper interrupt until one block is finished
void simulate_serial();

// Print information about the most recently inserted block
void printBlock();

//printer for grbl serial port output
void grbl_out(uint8_t char_out);

#endif
