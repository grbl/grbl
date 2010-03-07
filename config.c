/*
  config.c - eeprom and compile time configuration handling 
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

#include <avr/io.h>
#include <math.h>
#include "nuts_bolts.h"
#include "config.h"
#include "eeprom.h"
#include "wiring_serial.h"
#include <avr/pgmspace.h>

void reset_settings() {
  settings.steps_per_mm[0] = X_STEPS_PER_MM;
  settings.steps_per_mm[1] = Y_STEPS_PER_MM;
  settings.steps_per_mm[2] = Z_STEPS_PER_MM;
  settings.pulse_microseconds = STEP_PULSE_MICROSECONDS;
  settings.default_feed_rate = DEFAULT_FEEDRATE;
  settings.default_seek_rate = RAPID_FEEDRATE;
  settings.mm_per_arc_segment = MM_PER_ARC_SEGMENT;
  settings.invert_mask = STEPPING_INVERT_MASK;
}

void dump_settings() {
  printPgmString(PSTR("$0 = ")); printFloat(settings.steps_per_mm[0]);
  printPgmString(PSTR(" (steps/mm x)\r\n$1 = ")); printFloat(settings.steps_per_mm[1]);
  printPgmString(PSTR(" (steps/mm y)\r\n$2 = ")); printFloat(settings.steps_per_mm[2]);
  printPgmString(PSTR(" (steps/mm z)\r\n$3 = ")); printInteger(settings.pulse_microseconds);
  printPgmString(PSTR(" (microseconds step pulse)\r\n$4 = ")); printFloat(settings.default_feed_rate);
  printPgmString(PSTR(" (mm/sec default feed rate)\r\n$5 = ")); printFloat(settings.default_seek_rate);
  printPgmString(PSTR(" (mm/sec default seek rate)\r\n$6 = ")); printFloat(settings.mm_per_arc_segment);
  printPgmString(PSTR(" (mm/arc segment)\r\n$7 = ")); printInteger(settings.invert_mask); 
  printPgmString(PSTR(" (step port invert mask. binary = ")); printIntegerInBase(settings.invert_mask, 2);  
  printPgmString(PSTR(")\r\n\r\n'$x=value' to set parameter or just '$' to dump current settings\r\n"));
}

int read_settings() {
  // Check version-byte of eeprom
  uint8_t version = eeprom_get_char(0);  
  if (version != SETTINGS_VERSION) { return(FALSE); }
  // Read settings-record and check checksum
  if (!(memcpy_from_eeprom_with_checksum((char*)&settings, 1, sizeof(struct Settings)))) {
    return(FALSE);
  }
  return(TRUE);
}

void write_settings() {
  eeprom_put_char(0, SETTINGS_VERSION);
  memcpy_to_eeprom_with_checksum(1, (char*)&settings, sizeof(struct Settings));
}

// A helper method to set settings from command line
void store_setting(int parameter, double value) {
  switch(parameter) {
    case 0: case 1: case 2:
    settings.steps_per_mm[parameter] = value; break;
    case 3: settings.pulse_microseconds = round(value); break;
    case 4: settings.default_feed_rate = value; break;
    case 5: settings.default_seek_rate = value; break;
    case 6: settings.mm_per_arc_segment = value; break;
    case 7: settings.invert_mask = trunc(value); break;
    default: 
    printPgmString(PSTR("Unknown parameter\r\n"));
    return;
  }
  write_settings();
  printPgmString(PSTR("Stored new setting\r\n"));
}

void config_init() {
  if(read_settings()) {
    printPgmString(PSTR("'$' to dump current settings\r\n"));
  } else {
    printPgmString(("EEPROM blank. Rewrote default settings:\r\n"));
    reset_settings();
    write_settings();
    dump_settings();
  }
}
