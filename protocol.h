/*
  protocol.h - the serial protocol master control unit
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011-2012 Sungeun K. Jeon

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
#ifndef protocol_h
#define protocol_h

// Line buffer size from the serial input stream to be executed.
// NOTE: Not a problem except for extreme cases, but the line buffer size can be too small
// and g-code blocks can get truncated. Officially, the g-code standards support up to 256
// characters. In future versions, this will be increased, when we know how much extra
// memory space we can invest into here or we re-write the g-code parser not to have his 
// buffer.
#define LINE_BUFFER_SIZE 50

// Define Grbl status codes.
#define STATUS_OK 0
#define STATUS_BAD_NUMBER_FORMAT 1
#define STATUS_EXPECTED_COMMAND_LETTER 2
#define STATUS_UNSUPPORTED_STATEMENT 3
#define STATUS_FLOATING_POINT_ERROR 4
#define STATUS_MODAL_GROUP_VIOLATION 5
#define STATUS_INVALID_STATEMENT 6
#define STATUS_HARD_LIMIT 7
#define STATUS_SETTING_DISABLED 8
#define STATUS_SETTING_STEPS_NEG 9
#define STATUS_SETTING_STEP_PULSE_MIN 10
#define STATUS_SETTING_READ_FAIL 11

// Define Grbl misc message codes
#define MESSAGE_SYSTEM_ALARM 0
#define MESSAGE_HOMING_ENABLE 1


// Initialize the serial protocol
void protocol_init();

// Read command lines from the serial port and execute them as they
// come in. Blocks until the serial buffer is emptied. 
void protocol_process();

// Executes one line of input according to protocol
uint8_t protocol_execute_line(char *line);

// Checks and executes a runtime command at various stop points in main program
void protocol_execute_runtime();

// Prints Grbl's status messages.
void protocol_status_message(uint8_t status_code);

// Prints any misc messages.
void protocol_misc_message(uint8_t message_code);

#endif
