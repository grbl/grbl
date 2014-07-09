/*
  serial.c - replacement for the modul of the same name in grbl
    Make sure the simulator reads from stdin and writes to stdout.
    Also print info about the last buffered block.

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

#include "../serial.h"
#include <stdio.h>
#include "simulator.h"
#include <stdio.h>


//prototypes for overridden functions
uint8_t orig_serial_read();

//used to inject a sleep in grbl main loop, 
// ensures hardware simulator gets some cycles in "parallel"
uint8_t serial_read() {
  platform_sleep(0);
  return orig_serial_read();
}


void simulate_write_interrupt(){
  while (UCSR0B & (1<<UDRIE0)){
	 interrupt_SERIAL_UDRE();
	 grbl_out(UDR0);
  }
}

void simulate_read_interrupt(){
  uint8_t char_in = platform_poll_stdin();
  if (char_in) {
	 UDR0 = char_in;
	 //EOF or CTRL-F to exit
	 if (UDR0 == EOF || UDR0 == 0xFF || UDR0 == 0x06 ) {
		sim.exit = 1;
	 }
	 //debugging
	 if (UDR0 == '%') { 
		printf("%ld %f\n",sim.masterclock,(double)sim.sim_time);
	 }
	 interrupt_SERIAL_RX();
  }
}


extern volatile uint8_t rx_buffer_head;
extern volatile uint8_t rx_buffer_tail;
void simulate_serial(){
  simulate_write_interrupt();
  uint8_t head = rx_buffer_head+1;
  if (head==RX_BUFFER_SIZE) { head = 0; }
  if (head!=rx_buffer_tail) {
	 simulate_read_interrupt();
  }
}
