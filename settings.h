/*
  settings.h - eeprom configuration handling 
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

#ifndef settings_h
#define settings_h


#include <math.h>
#include <inttypes.h>

#define GRBL_VERSION "0.7d"

// Version of the EEPROM data. Will be used to migrate existing data from older versions of Grbl
// when firmware is upgraded. Always stored in byte 0 of eeprom
#define SETTINGS_VERSION 4

// Current global settings (persisted in EEPROM from byte 1 onwards)
typedef struct {
  double steps_per_mm[3];
  uint8_t microsteps;
  uint8_t pulse_microseconds;
  double default_feed_rate;
  double default_seek_rate;
  uint8_t invert_mask;
  double mm_per_arc_segment;
  double acceleration;
  double junction_deviation;
  // This parameter sets the delay time before disabling the steppers after the final block of movement.
  // A short delay ensures the steppers come to a complete stop and the residual inertial force in the 
  // CNC axes don't cause the axes to drift off position. This is particularly important when manually 
  // entering g-code into grbl, i.e. locating part zero or simple manual machining. If the axes drift,
  // grbl has no way to know this has happened, since stepper motors are open-loop control. Depending
  // on the machine, this parameter may need to be larger or smaller than the default time.
  uint16_t idle_lock_time;
} settings_t;
extern settings_t settings;

// Initialize the configuration subsystem (load settings from EEPROM)
void settings_init();

// Print current settings
void settings_dump();

// Handle settings command
uint8_t settings_execute_line(char *line);

// A helper method to set new settings from command line
void settings_store_setting(int parameter, double value);

#endif
