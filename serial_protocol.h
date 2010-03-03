/*
  serial_protocol.h - the serial protocol master control unit
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
#ifndef serial_h
#define serial_h

// A string to let the client know we are ready for a new command
#define PROMPT "\r\n>>>"
// A character to acknowledge that the execution has started
#define EXECUTION_MARKER '~'

// Initialize the serial protocol
void sp_init();
// Read command lines from the serial port and execute them as they
// come in. Blocks until the serial buffer is emptied. 
void sp_process();

#endif
