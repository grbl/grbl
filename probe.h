/*
  probe.h - code pertaining to probing methods
  Part of Grbl v0.9

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
  
#ifndef probe_h
#define probe_h 

// Values that define the probing state machine.  
#define PROBE_OFF     0 // No probing. (Must be zero.)
#define PROBE_ACTIVE  1 // Actively watching the input pin.

// Probe direction and error modes
#define PROBE_AWAY 2 // G38.4, G38.5
#define PROBE_NO_ERROR 4 // G38.3, G38.5

#define PROBE_AWAY_BIT 1
#define PROBE_NO_ERROR_BIT 2

// Probe pin initialization routine.
void probe_init();

// Returns probe pin state.
uint8_t probe_get_state(uint8_t mode);

uint8_t probe_errors_enabled(uint8_t mode);

void probe_finalize();

// Monitors probe pin state and records the system position when detected. Called by the
// stepper ISR per ISR tick.
void probe_state_monitor();

#endif
