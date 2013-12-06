/*
  runtime.c - replacement for the modul of the same name in grbl
    Run time commands are not processed in the simulator.
    Instead, the execute_runtime() is used as a hook to handle stepper simulation
    and printing of simulation results.

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

#include "simulator.h"
#include <stdio.h>

void orig_protocol_execute_runtime(void);

// replacement for original execute_runtime as a hook to print blocks as they are generated
// and to control simulation of buffered blocks
void protocol_execute_runtime(void) {
  orig_protocol_execute_runtime();
  //printf("printBlock():\n");
  printBlock();
  //printf("handle_buffer():\n");
  handle_buffer();
}
