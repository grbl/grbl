/*
  nuts_bolts.h - Header file for shared definitions, variables, and functions
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

#ifndef nuts_bolts_h
#define nuts_bolts_h

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "defaults.h"

#define false 0
#define true 1

#define N_AXIS 3 // Number of axes
#define X_AXIS 0 // Axis indexing value
#define Y_AXIS 1
#define Z_AXIS 2

#define MM_PER_INCH (25.40)
#define INCH_PER_MM (0.0393701)

// Useful macros
#define clear_vector(a) memset(a, 0, sizeof(a))
#define clear_vector_float(a) memset(a, 0.0, sizeof(float)*N_AXIS)
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))

// Bit field and masking macros
#define bit(n) (1 << n) 
#define bit_true(x,mask) (x |= mask)
#define bit_false(x,mask) (x &= ~mask)
#define bit_toggle(x,mask) (x ^= mask)
#define bit_istrue(x,mask) ((x & mask) != 0)
#define bit_isfalse(x,mask) ((x & mask) == 0)

// Define system executor bit map. Used internally by runtime protocol as runtime command flags, 
// which notifies the main program to execute the specified runtime command asynchronously.
// NOTE: The system executor uses an unsigned 8-bit volatile variable (8 flag limit.) The default
// flags are always false, so the runtime protocol only needs to check for a non-zero value to 
// know when there is a runtime command to execute.
#define EXEC_STATUS_REPORT  bit(0) // bitmask 00000001
#define EXEC_CYCLE_START    bit(1) // bitmask 00000010
#define EXEC_CYCLE_STOP     bit(2) // bitmask 00000100
#define EXEC_FEED_HOLD      bit(3) // bitmask 00001000
#define EXEC_RESET          bit(4) // bitmask 00010000
#define EXEC_ALARM          bit(5) // bitmask 00100000
#define EXEC_CRIT_EVENT     bit(6) // bitmask 01000000
// #define                  bit(7) // bitmask 10000000

// Define system state bit map. The state variable primarily tracks the individual functions
// of Grbl to manage each without overlapping. It is also used as a messaging flag for
// critical events.
#define STATE_IDLE       0 // Must be zero.
#define STATE_INIT       1 // Initial power up state.
#define STATE_QUEUED     2 // Indicates buffered blocks, awaiting cycle start.
#define STATE_CYCLE      3 // Cycle is running
#define STATE_HOLD       4 // Executing feed hold
#define STATE_HOMING     5 // Performing homing cycle
#define STATE_ALARM      6 // In alarm state. Locks out all g-code processes. Allows settings access.
#define STATE_CHECK_MODE 7 // G-code check mode. Locks out planner and motion only.
// #define STATE_JOG     8 // Jogging mode is unique like homing.

// Define global system variables
typedef struct {
  uint8_t abort;                 // System abort flag. Forces exit back to main loop for reset.
  uint8_t state;                 // Tracks the current state of Grbl.
  volatile uint8_t execute;      // Global system runtime executor bitflag variable. See EXEC bitmasks.
  int32_t position[N_AXIS];      // Real-time machine (aka home) position vector in steps. 
                                 // NOTE: This may need to be a volatile variable, if problems arise.   
  uint8_t auto_start;            // Planner auto-start flag. Toggled off during feed hold. Defaulted by settings.
} system_t;
extern system_t sys;

// Read a floating point value from a string. Line points to the input buffer, char_counter 
// is the indexer pointing to the current character of the line, while float_ptr is 
// a pointer to the result variable. Returns true when it succeeds
int read_float(char *line, uint8_t *char_counter, float *float_ptr);

// Delays variable-defined milliseconds. Compiler compatibility fix for _delay_ms().
void delay_ms(uint16_t ms);

// Delays variable-defined microseconds. Compiler compatibility fix for _delay_us().
void delay_us(uint32_t us);

// Syncs Grbl's gcode and planner position variables with the system position.
void sys_sync_current_position();

#endif
