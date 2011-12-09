/*
  nuts_bolts.h - Header file for shared definitions, variables, and functions
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

#ifndef nuts_bolts_h
#define nuts_bolts_h
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define false 0
#define true 1

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define clear_vector(a) memset(a, 0, sizeof(a))
#define clear_vector_double(a) memset(a, 0.0, sizeof(a))
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))

// Define system state bit map. Used internally by runtime protocol as runtime command flags.
// NOTE: The system state is an unsigned 8-bit volatile variable and has a 8 flag limit. The default
// flags are always false, so the runtime protocol only needs to check for a non-zero state value to 
// know when there is a runtime command to execute.
#define BIT_STATUS_REPORT    1 // bit 00000001
#define BIT_CYCLE_START      2 // bit 00000010
#define BIT_FEED_HOLD        4 // bit 00000100
#define BIT_RESET            8 // bit 00001000
#define BIT_REPLAN_CYCLE    16 // bit 00010000
// #define                     32 // bit 00100000
// #define                     64 // bit 01000000
// #define                    128 // bit 10000000

// Define global system variables
extern uint8_t sys_abort; // Global system abort flag
extern volatile uint8_t sys_state; // Global system state variable

// Read a floating point value from a string. Line points to the input buffer, char_counter 
// is the indexer pointing to the current character of the line, while double_ptr is 
// a pointer to the result variable. Returns true when it succeeds
int read_double(char *line, uint8_t *char_counter, double *double_ptr);

#endif
