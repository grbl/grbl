/*
  system.h - Header for system level commands and real-time processes
  Part of Grbl

  Copyright (c) 2014 Sungeun K. Jeon  

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

#ifndef system_h
#define system_h

// Define system header files and standard libraries used by Grbl
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <math.h>
#include <inttypes.h>    
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

// Define Grbl configuration and shared header files
#include "config.h"
#include "defaults.h"
#include "cpu_map.h"
#include "nuts_bolts.h"


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
#define STATE_IDLE       0      // Must be zero. No flags.
#define STATE_ALARM      bit(0) // In alarm state. Locks out all g-code processes. Allows settings access.
#define STATE_CHECK_MODE bit(1) // G-code check mode. Locks out planner and motion only.
#define STATE_HOMING     bit(2) // Performing homing cycle
#define STATE_QUEUED     bit(3) // Indicates buffered blocks, awaiting cycle start.
#define STATE_CYCLE      bit(4) // Cycle is running
#define STATE_HOLD       bit(5) // Executing feed hold
// #define STATE_JOG     bit(6) // Jogging mode is unique like homing.


// Define global system variables
typedef struct {
  uint8_t abort;                 // System abort flag. Forces exit back to main loop for reset.
  uint8_t state;                 // Tracks the current state of Grbl.
  volatile uint8_t execute;      // Global system runtime executor bitflag variable. See EXEC bitmasks.
  uint8_t homing_axis_lock;
  int32_t position[N_AXIS];      // Real-time machine (aka home) position vector in steps. 
                                 // NOTE: This may need to be a volatile variable, if problems arise.                             
  uint8_t auto_start;            // Planner auto-start flag. Toggled off during feed hold. Defaulted by settings.
  volatile uint8_t probe_state;   // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
  int32_t probe_position[N_AXIS]; // Last probe position in machine coordinates and steps.
} system_t;
extern system_t sys;


// Initialize the serial protocol
void system_init();

// Executes an internal system command, defined as a string starting with a '$'
uint8_t system_execute_line(char *line);

// Checks and executes a runtime command at various stop points in main program
void system_execute_runtime();

// Execute the startup script lines stored in EEPROM upon initialization
void system_execute_startup(char *line);

#endif
