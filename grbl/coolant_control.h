/*
  coolant_control.h - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

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

#ifndef coolant_control_h
#define coolant_control_h

#define COOLANT_NO_SYNC     false
#define COOLANT_FORCE_SYNC  true

#define COOLANT_STATE_DISABLE   0  // Must be zero
#define COOLANT_STATE_FLOOD     PL_COND_FLAG_COOLANT_FLOOD
#define COOLANT_STATE_MIST      PL_COND_FLAG_COOLANT_MIST


// Initializes coolant control pins.
void coolant_init();

// Returns current coolant output state. Overrides may alter it from programmed state.
uint8_t coolant_get_state();

// Immediately disables coolant pins.
void coolant_stop();

// Sets the coolant pins according to state specified.
void coolant_set_state(uint8_t mode);

// G-code parser entry-point for setting coolant states. Checks for and executes additional conditions.
void coolant_sync(uint8_t mode);

#endif
