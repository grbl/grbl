/*
  settings.c - eeprom configuration handling 
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#include <avr/io.h>
#include <math.h>
#include "nuts_bolts.h"
#include "settings.h"
#include "eeprom.h"
#include "wiring_serial.h"
#include <avr/pgmspace.h>

settings_t settings;

// Version 1 outdated settings record
typedef struct {
  double steps_per_mm[3];
  uint8_t microsteps;
  uint8_t pulse_microseconds;
  double default_feed_rate;
  double default_seek_rate;
  uint8_t invert_mask;
  double mm_per_arc_segment;
} settings_v1_t;

void settings_reset() {
  settings.steps_per_mm[X_AXIS] = DEFAULT_X_STEPS_PER_MM;
  settings.steps_per_mm[Y_AXIS] = DEFAULT_Y_STEPS_PER_MM;
  settings.steps_per_mm[Z_AXIS] = DEFAULT_Z_STEPS_PER_MM;
  settings.pulse_microseconds = DEFAULT_STEP_PULSE_MICROSECONDS;
  settings.default_feed_rate = DEFAULT_FEEDRATE;
  settings.default_seek_rate = DEFAULT_RAPID_FEEDRATE;
  settings.acceleration = DEFAULT_ACCELERATION;
  settings.mm_per_arc_segment = DEFAULT_MM_PER_ARC_SEGMENT;
  settings.invert_mask = DEFAULT_STEPPING_INVERT_MASK;
  settings.max_jerk = DEFAULT_MAX_JERK;
}

void settings_dump() {
  printPgmString(PSTR("$0 = ")); printFloat(settings.steps_per_mm[X_AXIS]);
  printPgmString(PSTR(" (steps/mm x)\r\n$1 = ")); printFloat(settings.steps_per_mm[Y_AXIS]);
  printPgmString(PSTR(" (steps/mm y)\r\n$2 = ")); printFloat(settings.steps_per_mm[Z_AXIS]);
  printPgmString(PSTR(" (steps/mm z)\r\n$3 = ")); printInteger(settings.pulse_microseconds);
  printPgmString(PSTR(" (microseconds step pulse)\r\n$4 = ")); printFloat(settings.default_feed_rate);
  printPgmString(PSTR(" (mm/min default feed rate)\r\n$5 = ")); printFloat(settings.default_seek_rate);
  printPgmString(PSTR(" (mm/min default seek rate)\r\n$6 = ")); printFloat(settings.mm_per_arc_segment);
  printPgmString(PSTR(" (mm/arc segment)\r\n$7 = ")); printInteger(settings.invert_mask); 
  printPgmString(PSTR(" (step port invert mask. binary = ")); printIntegerInBase(settings.invert_mask, 2);  
  printPgmString(PSTR(")\r\n$8 = ")); printFloat(settings.acceleration);
  printPgmString(PSTR(" (acceleration in mm/sec^2)\r\n$9 = ")); printFloat(settings.max_jerk);
  printPgmString(PSTR(" (max instant cornering speed change in delta mm/min)"));
  printPgmString(PSTR("\r\n'$x=value' to set parameter or just '$' to dump current settings\r\n"));
}

void write_settings() {
  eeprom_put_char(0, SETTINGS_VERSION);
  memcpy_to_eeprom_with_checksum(1, (char*)&settings, sizeof(settings_t));
}

int read_settings() {
  // Check version-byte of eeprom
  uint8_t version = eeprom_get_char(0);
  
  if (version == SETTINGS_VERSION) {
    // Read settings-record and check checksum
    if (!(memcpy_from_eeprom_with_checksum((char*)&settings, 1, sizeof(settings_t)))) {
      return(FALSE);
    }
  } else if (version == 1) {
    // Migrate from old settings version
    if (!(memcpy_from_eeprom_with_checksum((char*)&settings, 1, sizeof(settings_v1_t)))) {
      return(FALSE);
    }
    settings.acceleration = DEFAULT_ACCELERATION;
    settings.max_jerk = DEFAULT_MAX_JERK;
  } else {      
    return(FALSE);
  }
  return(TRUE);
}

// A helper method to set settings from command line
void settings_store_setting(int parameter, double value) {
  switch(parameter) {
    case 0: case 1: case 2:
    settings.steps_per_mm[parameter] = value; break;
    case 3: settings.pulse_microseconds = round(value); break;
    case 4: settings.default_feed_rate = value; break;
    case 5: settings.default_seek_rate = value; break;
    case 6: settings.mm_per_arc_segment = value; break;
    case 7: settings.invert_mask = trunc(value); break;
    case 8: settings.acceleration = value; break;
    case 9: settings.max_jerk = fabs(value); break;
    default: 
      printPgmString(PSTR("Unknown parameter\r\n"));
      return;
  }
  write_settings();
  printPgmString(PSTR("Stored new setting\r\n"));
}

// Initialize the config subsystem
void settings_init() {
  if(read_settings()) {
    printPgmString(PSTR("'$' to dump current settings\r\n"));
  } else {
    printPgmString(PSTR("Warning: Failed to read EEPROM settings. Using defaults.\r\n"));
    settings_reset();
    write_settings();
    settings_dump();
  }
}
