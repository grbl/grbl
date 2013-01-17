/*
  simulator.c - functions to simulate how the buffer is emptied and the
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

#include <stdio.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <stdio.h>
#include "../stepper.h"
#include "../planner.h"
#include "../nuts_bolts.h"
#include "simulator.h"

// This variable is needed to determine if execute_runtime() is called in a loop
// waiting for the buffer to empty, as in plan_synchronize()
// it is reset in serial_write() because this is certainly called at the end of
// every command processing
int runtime_second_call= 0;


// Current time of the stepper simulation
double sim_time= 0.0;
// Next time the status of stepper values should be printed
double next_print_time= 0.0;
// Minimum time step for printing stepper values. Given by user via command line
double step_time= 0.0;

// global system variable structure for position etc.
system_t sys;
int block_position[]= {0,0,0};
uint8_t print_comment= 1;
uint8_t end_of_block= 1;
uint32_t block_number= 0;

// Output file handles set by main program
FILE *block_out_file;
FILE *step_out_file;

// dummy port variables
uint8_t stepping_ddr;
uint8_t stepping_port;
uint8_t spindle_ddr;
uint8_t spindle_port;
uint8_t limit_ddr;
uint8_t limit_port;
uint8_t limit_int_reg;
uint8_t pinout_ddr;
uint8_t pinout_port;
uint8_t pinout_int_reg;
uint8_t coolant_flood_ddr;
uint8_t coolant_flood_port;

extern block_t block_buffer[BLOCK_BUFFER_SIZE];  // A ring buffer for motion instructions
extern uint8_t block_buffer_head;       // Index of the next block to be pushed
extern uint8_t block_buffer_tail;       // Index of the block to process now


// Stub of the timer interrupt function from stepper.c
void interrupt_TIMER2_COMPA_vect();

// Call the stepper interrupt until one block is finished
void sim_stepper() {
  //printf("sim_stepper()\n");
  block_t *current_block= plan_get_current_block();

  // If the block buffer is empty, call the stepper interrupt one last time
  // to let it handle sys.cycle_start etc.
  if(current_block==NULL) {
	  interrupt_TIMER2_COMPA_vect();
	  return;
  }

  while(current_block==plan_get_current_block()) {
    sim_time+= get_step_time();
    interrupt_TIMER2_COMPA_vect();

    // Check to see if we should print some info
    if(step_time>0.0) {
    	if(sim_time>=next_print_time) {
    	  if(end_of_block) {
    		end_of_block= 0;
    		fprintf(step_out_file, "# block number %d\n", block_number);
    	  }
          fprintf(step_out_file, "%20.15f, %d, %d, %d\n", sim_time, sys.position[X_AXIS], sys.position[Y_AXIS], sys.position[Z_AXIS]);

          // Make sure the simulation time doesn't get ahead of next_print_time
          while(next_print_time<sim_time) next_print_time+= step_time;
    	}
    }

  }
  // always print stepper values at the end of a block
  if(step_time>0.0) {
	fprintf(step_out_file, "%20.15f, %d, %d, %d\n", sim_time, sys.position[X_AXIS], sys.position[Y_AXIS], sys.position[Z_AXIS]);
	end_of_block= 1;
	block_number++;
  }
}

// Returns the index of the previous block in the ring buffer
uint8_t prev_block_index(uint8_t block_index)
{
  if (block_index == 0) { block_index = BLOCK_BUFFER_SIZE; }
  block_index--;
  return(block_index);
}

block_t *get_block_buffer();
uint8_t get_block_buffer_head();
uint8_t get_block_buffer_tail();

block_t *plan_get_recent_block() {
  if (get_block_buffer_head() == get_block_buffer_tail()) { return(NULL); }
  return(get_block_buffer()+prev_block_index(get_block_buffer_head()));
}


// Print information about the most recently inserted block
// but only once!
void printBlock() {
  block_t *b;
  static block_t *last_block;

  //printf("printBlock()\n");

  b= plan_get_recent_block();
  if(b!=last_block && b!=NULL) {
	//fprintf(block_out_file,"%s\n", line);
    //fprintf(block_out_file,"  block: ");
    if(b->direction_bits & (1<<X_DIRECTION_BIT)) block_position[0]-= b->steps_x;
    else block_position[0]+= b->steps_x;
    fprintf(block_out_file,"%d, ", block_position[0]);

    if(b->direction_bits & (1<<Y_DIRECTION_BIT)) block_position[1]-= b->steps_y;
    else block_position[1]+= b->steps_y;
    fprintf(block_out_file,"%d, ", block_position[1]);

    if(b->direction_bits & (1<<Z_DIRECTION_BIT)) block_position[2]-= b->steps_z;
    else block_position[2]+= b->steps_z;
    fprintf(block_out_file,"%d, ", block_position[2]);

    fprintf(block_out_file,"%f", b->entry_speed_sqr);
    fprintf(block_out_file,"\n");

    last_block= b;
  }
}

// The simulator assumes that grbl is fast enough to keep the buffer full.
// Thus, the stepper interrupt is only called when the buffer is full and then only to
// finish one block.
// Only when plan_synchronize() wait for the whole buffer to clear, the stepper interrupt
// to finish all pending moves.
void handle_buffer() {
  // runtime_second_call is reset by serial_write() after every command.
  // Only when execute_runtime() is called repeatedly by plan_synchronize()
  // runtime_second_call will be incremented above 2
  //printf("handle_buffer()\n");
  if(plan_check_full_buffer() || runtime_second_call>2) {
    sim_stepper(step_out_file);
  } else {
    runtime_second_call++;
  }
}

double get_step_time() {
/* code for the old stepper algorithm
  uint16_t ceiling;
  uint16_t prescaler;
  uint32_t actual_cycles;
  uint8_t invalid_prescaler= 0;

  prescaler= ((TCCR1B>>CS10) & 0x07) - 1;
  ceiling= OCR1A;

  switch(prescaler) {
    case 0:
      actual_cycles= ceiling;
	  break;
    case 1:
      actual_cycles= ceiling * 8L;
	  break;
    case 2:
      actual_cycles = ceiling * 64L;
	  break;
    case 3:
      actual_cycles = ceiling * 256L;
	  break;
    case 4:
      actual_cycles = ceiling * 1024L;
	  break;
    default:
    	invalid_prescaler= 1;
  }

  if(invalid_prescaler) return 12345.0;
  else return (double)actual_cycles/F_CPU;*/
  return (double)((ocr2a+1)*8)/(double)(F_CPU);
}
