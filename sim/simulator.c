/*
  simulator.c - functions to simulate how the buffer is emptied and the
    stepper interrupt is called

  Part of Grbl Simulator

  Copyright (c) 2012-2014 Jens Geisler, Adam Shelly

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
#include "avr/interrupt.h" //for registers and isr declarations.
#include "eeprom.h"


int block_position[N_AXIS]= {0}; //step count after most recently planned block
uint32_t block_number= 0;

sim_vars_t sim={0};

//local prototypes 
void print_steps(bool force);


//setup 
void init_simulator(float time_multiplier) {

  //register the interrupt handlers we actually use.
  compa_vect[1] = interrupt_TIMER1_COMPA_vect;
  ovf_vect[0] = interrupt_TIMER0_OVF_vect;
#ifdef STEP_PULSE_DELAY
  compa_vect[0] = interrupt_TIMER0_COMPA_vect;
#endif
#ifdef ENABLE_SOFTWARE_DEBOUNCE
  wdt_vect = interrupt_WDT_vect;
#endif
  pc_vect = interrupt_LIMIT_INT_vect;


  sim.next_print_time = args.step_time;
  sim.speedup = time_multiplier;
  sim.baud_ticks = (int)((double)F_CPU*8/BAUD_RATE); //ticks per byte
}



//shutdown simulator - close open files
int shutdown_simulator(uint8_t exitflag) {
  fclose(args.block_out_file);
  print_steps(1);
  fclose(args.step_out_file);
  fclose(args.grbl_out_file);
  eeprom_close();
  return 1/(!exitflag);  //force exception, since avr_main() has no returns.
}



void simulate_hardware(bool do_serial){

  //do one tick
  sim.masterclock++;
  sim.sim_time = (float)sim.masterclock/F_CPU;

  timer_interrupts();
  
  if (do_serial) simulate_serial();

  //TODO:
  //  check limit pins,  call pinchange interrupt if enabled
  //  can ignore pinout int vect - hw start/hold not supported

}

//runs the hardware simulator at the desired rate until sim.exit is set
void sim_loop(){
  uint64_t simulated_ticks=0;
  uint32_t ns_prev = platform_ns();
  uint64_t next_byte_tick = F_CPU;   //wait 1 sec before reading IO.

  while (!sim.exit || sys.state>2 ) { //don't quit until idle

   if (sim.speedup) {
    //calculate how many ticks to do.
    uint32_t ns_now = platform_ns();
    uint32_t ns_elapsed = (ns_now-ns_prev)*sim.speedup; //todo: try multipling nsnow
    simulated_ticks += F_CPU/1e9*ns_elapsed;
    ns_prev = ns_now;
   }
   else {
    simulated_ticks++;  //as fast as possible
   }
    
   while (sim.masterclock < simulated_ticks){

    //only read serial port as fast as the baud rate allows
    bool read_serial = (sim.masterclock >= next_byte_tick);

    //do low level hardware
    simulate_hardware(read_serial);

    //print the steps. 
    //For further decoupling, could maintain own counter of STEP_PORT pulses, 
      // print that instead of sys.position.
    print_steps(0);  
    
    if (read_serial){
      next_byte_tick+=sim.baud_ticks;
      //recent block can only change after input, so check here.
      printBlock();
    }

    //TODO:
    //  set limit pins based on position,
    //  set probe pin when probing.
    //  if VARIABLE_SPINDLE, measure pwm pin to report speed?
   }

   platform_sleep(0); //yield
  }
}


//show current position in steps
void print_steps(bool force)
{ 
  static plan_block_t* printed_block = NULL;
  plan_block_t* current_block = plan_get_current_block();

  if (sim.next_print_time == 0.0) { return; }  //no printing
  if (current_block != printed_block ) {
   //new block. 
   if (block_number) { //print values from the end of prev block
    fprintf(args.step_out_file, "%20.15f %d, %d, %d\n", sim.sim_time, sys.position[X_AXIS], sys.position[Y_AXIS], sys.position[Z_AXIS]);
   }
   printed_block = current_block;
   if (current_block == NULL) { return; }
   // print header
   fprintf(args.step_out_file, "# block number %d\n", block_number++);
  }
  //print at correct interval while executing block
  else if ((current_block && sim.sim_time>=sim.next_print_time) || force ) {
   fprintf(args.step_out_file, "%20.15f %d, %d, %d\n", sim.sim_time, sys.position[X_AXIS], sys.position[Y_AXIS], sys.position[Z_AXIS]);
   fflush(args.step_out_file);

   //make sure the simulation time doesn't get ahead of next_print_time
   while (sim.next_print_time<=sim.sim_time) sim.next_print_time += args.step_time;
  }
}


//Functions for peeking inside planner state:
plan_block_t *get_block_buffer();
uint8_t get_block_buffer_head();
uint8_t get_block_buffer_tail();

// Returns the index of the previous block in the ring buffer
uint8_t prev_block_index(uint8_t block_index)
{
  if (block_index == 0) { block_index = BLOCK_BUFFER_SIZE; }
  block_index--;
  return(block_index);
}

plan_block_t *plan_get_recent_block() {
  if (get_block_buffer_head() == get_block_buffer_tail()) { return(NULL); }
  return(get_block_buffer()+prev_block_index(get_block_buffer_head()));
}

// Print information about the most recently inserted block
// but only once!
void printBlock() {
  plan_block_t *b;
  static plan_block_t *last_block;

  b= plan_get_recent_block();
  if(b!=last_block && b!=NULL) {
   int i;
   for (i=0;i<N_AXIS;i++){
    if(b->direction_bits & get_direction_mask(i)) block_position[i]-= b->steps[i];
    else block_position[i]+= b->steps[i];
    fprintf(args.block_out_file,"%d, ", block_position[i]);
   }
   fprintf(args.block_out_file,"%f", b->entry_speed_sqr);
   fprintf(args.block_out_file,"\n");
   fflush(args.block_out_file); //TODO: needed?

    last_block= b;
  }
}


//printer for grbl serial port output
void grbl_out(uint8_t data){
  static uint8_t buf[128]={0};
  static uint8_t len=0;
  static bool continuation = 0;

  buf[len++]=data;
  if(data=='\n' || data=='\r' || len>=127) {
   if (args.comment_char && !continuation){
    fprintf(args.grbl_out_file,"%c ",args.comment_char);
   }
   buf[len]=0;
   fprintf(args.grbl_out_file,"%s",buf);
   continuation = (len>=128); //print comment on next line unless we are only printing to avoid buffer overflow)
   len=0;
  }
}
