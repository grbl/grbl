/*
  main.c - main grbl simulator program

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

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include "../planner.h"
#include "../stepper.h"
#include "../gcode.h"
#include "../protocol.h"
#include "../nuts_bolts.h"
#include "../settings.h"
#include "../spindle_control.h"
#include "../limits.h"
#include "../coolant_control.h"
#include "simulator.h"


arg_vars_t args;
const char* progname;

int usage(const char* badarg){
  if (badarg){
	 printf("Unrecognized option %s\n",badarg);
  }
  printf("Usage: \n"
			"%s [options] [time_step] [block_file]\n"
			"  Options:\n"
			"    -r <report time>   : minimum time step for printing stepper values. Default=0=no print.\n"
			"    -t <time factor>   : multiplier to realtime clock. Default=1. (needs work)\n"
			"    -g <response file> : file to report responses from grbl.  default = stdout\n"
			"    -b <block file>    : file to report each block executed.  default = stdout\n"
			"    -s <step file>     : file to report each step executed.  default = stderr\n"
			"    -c<comment_char>   : character to print before each line from grbl.  default = '#'\n"
			"    -n                 : no comments before grbl response lines.\n"
			"    -h                 : this help.\n"
			"\n  <time_step> and <block_file> can be specifed with option flags or positional parameters\n"
			"\n  ^-F to shutdown cleanly\n\n",
			progname);
  return -1;
}

//prototype for renamed original main function
int avr_main(void);
//wrapper for thread interface
PLAT_THREAD_FUNC(avr_main_thread,exit){
  avr_main();
  return NULL;
}

int main(int argc, char *argv[]) {
  uint32_t tick_rate=1;
  int positional_args=0;

  //defaults
  args.step_out_file = stderr;
  args.block_out_file = stdout;
  args.grbl_out_file = stdout;
  args.comment_char = '#';

  args.step_time = 0.0;
  // Get the minimum time step for printing stepper values.
  // If not given or the command line cannot be parsed to a float than
  // step_time= 0.0; This means to not print stepper values at all

  progname = argv[0];
  while (argc>1) {
	  argv++;argc--;
	  if (argv[0][0] == '-'){
		 switch(argv[0][1]){
		 case 'c':  //set Comment char 
			args.comment_char = argv[0][2];
			break;
		 case 'n': //No comment char on grbl responses
			args.comment_char = 0;
			break;
		 case 't': //Tick rate
			argv++;argc--;
			tick_rate = atof(*argv);
			break;
		 case 'b': //Block file
			argv++;argc--;
			args.block_out_file = fopen(*argv,"w");
			break;
		 case 's': //Step out file.
			argv++;argc--;
			args.step_out_file = fopen(*argv,"w");
			break;
		 case 'g': //Grbl output
			argv++;argc--;
			args.grbl_out_file = fopen(*argv,"w");
			break;
		 case 'r':  //step_time for Reporting
			argv++;argc--;
			args.step_time= atof(*argv);
			break;
		 case 'h':
			return usage(NULL);
		 default:
			return usage(*argv);
		 }
	  }
	  else { //handle old positional argument interface
		 positional_args++;
		 switch(positional_args){
		 case 1:
			args.step_time= atof(*argv);
			break;
		 case 2:  //block out and grbl out to same file, like before.
			args.block_out_file = fopen(*argv,"w");
			args.grbl_out_file = args.block_out_file;
			break;
		 default:
			return usage(*argv);
		 }
	  }
  }

  // Make sure the output streams are flushed immediately.
  // This is important when using the simulator inside another application in parallel
  // to the real grbl.
  // Theoretically flushing could be limited to complete lines. Unfortunately Windows
  // does not know line buffered streams. So for now we stick to flushing every character.
  //setvbuf(stdout, NULL, _IONBF, 1);
  //setvbuf(stderr, NULL, _IONBF, 1);
  //( Files are now closed cleanly when sim gets EOF or CTRL-F.)
  platform_init(); 

  init_simulator(tick_rate);

  //launch a thread with the original grbl code.
  plat_thread_t*th = platform_start_thread(avr_main_thread); 
  if (!th){
	 printf("Fatal: Unable to start hardware thread.\n");
	 exit(-5);
  }

  //All the stream io and interrupt happen in this thread.
  sim_loop();

  platform_kill_thread(th); //need force kill since original main has no return.

  // Graceful exit
  shutdown_simulator(0);
  platform_terminate();

  exit(EXIT_SUCCESS);
}
