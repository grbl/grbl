/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Sungeun K. Jeon

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

/* This code was initially inspired by the wiring_serial module by David A. Mellis which
   used to be a part of the Arduino project. */ 

#include "config.h"
#include "serial.h"
#include "dev_misc.h"
#include "uart.h"

void serial_init(long baud)
{
	// already initialised before the start of the FreeRTOS scheduler
}

void serial_write(uint8_t data) {
	uart_putc(data);
}

uint8_t serial_read()
{
	uint8_t c = uart_getc(); 
	if(c == 0x00) {
		c = SERIAL_NO_DATA;
	}  
	return c;
}

void serial_reset_read_buffer() 
{
	
}

/* 		----THIS needs to be integrated in the FreeRTOS uart_receiver code!----------
 *				I dont know how critical they are...
 *
    // Pick off runtime command characters directly from the serial stream. These characters are
    // not passed into the buffer, but these set system state flag bits for runtime execution.
    // 		switch (data) {
		case CMD_STATUS_REPORT: sys_state |= BIT_STATUS_REPORT; break; // Set as true
		case CMD_CYCLE_START:   sys_state |= BIT_CYCLE_START; break; // Set as true
		case CMD_FEED_HOLD:     sys_state |= BIT_FEED_HOLD; break; // Set as true
		case CMD_RESET: 
       		// Immediately force stepper subsystem idle at an interrupt level.
		if (!(sys_state & BIT_RESET)) { // Force stop only first time.
			st_go_idle();  
		}
       		sys_state |= BIT_RESET; // Set as true
       		break;

         default : // Write character to buffer
        rx_buffer[rx_buffer_head] = data;
        rx_buffer_head = next_head;
*/

