/*
  settings.c - eeprom configuration handling 
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

#include <avr/io.h>
#include <math.h>
#include "nuts_bolts.h"
#include "settings.h"
#include "eeprom.h"
#include "print.h"
#include <avr/pgmspace.h>
#include "protocol.h"
#include "config.h"

settings_t settings;

// Version 1 outdated settings record
typedef struct {
  float steps_per_mm[3];
  uint8_t microsteps;
  uint8_t pulse_microseconds;
  float default_feed_rate;
  float default_seek_rate;
  uint8_t invert_mask;
  float mm_per_arc_segment;
} settings_v1_t;

// Version 2,3,4 outdated settings record
typedef struct {
  float steps_per_mm[3];
  uint8_t microsteps;
  uint8_t pulse_microseconds;
  float default_feed_rate;
  float default_seek_rate;
  uint8_t invert_mask;
  float mm_per_arc_segment;
  float acceleration;
  float junction_deviation;
} settings_v2_v4_t;

// Default settings (used when resetting eeprom-settings)
#define MICROSTEPS 8
#define DEFAULT_X_STEPS_PER_MM (94.488188976378*MICROSTEPS)
#define DEFAULT_Y_STEPS_PER_MM (94.488188976378*MICROSTEPS)
#define DEFAULT_Z_STEPS_PER_MM (94.488188976378*MICROSTEPS)
#define DEFAULT_STEP_PULSE_MICROSECONDS 30
#define DEFAULT_MM_PER_ARC_SEGMENT 0.1
#define DEFAULT_RAPID_FEEDRATE 500.0 // mm/min
#define DEFAULT_FEEDRATE 500.0
#define DEFAULT_ACCELERATION (DEFAULT_FEEDRATE*60*60/10.0) // mm/min^2
#define DEFAULT_JUNCTION_DEVIATION 0.05 // mm
#define DEFAULT_STEPPING_INVERT_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT))

// Developmental default settings
#define DEFAULT_REPORT_INCHES 0 // false
#define DEFAULT_AUTO_START 1 // true
#define DEFAULT_HARD_LIMIT_ENABLE 0  // false
#define DEFAULT_HOMING_ENABLE 0  // false
#define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
#define DEFAULT_HOMING_RAPID_FEEDRATE 250.0 // mm/min
#define DEFAULT_HOMING_FEEDRATE 50 // mm/min
#define DEFAULT_HOMING_DEBOUNCE_DELAY 100 // msec (0-65k)
#define DEFAULT_HOMING_PULLOFF 1 // mm
#define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-255)
#define DEFAULT_DECIMAL_PLACES 3

void settings_reset(bool reset_all) {
  // Reset all settings or only the migration settings to the new version.
  if (reset_all) {
    settings.steps_per_mm[X_AXIS] = DEFAULT_X_STEPS_PER_MM;
    settings.steps_per_mm[Y_AXIS] = DEFAULT_Y_STEPS_PER_MM;
    settings.steps_per_mm[Z_AXIS] = DEFAULT_Z_STEPS_PER_MM;
    settings.pulse_microseconds = DEFAULT_STEP_PULSE_MICROSECONDS;
    settings.default_feed_rate = DEFAULT_FEEDRATE;
    settings.default_seek_rate = DEFAULT_RAPID_FEEDRATE;
    settings.acceleration = DEFAULT_ACCELERATION;
    settings.mm_per_arc_segment = DEFAULT_MM_PER_ARC_SEGMENT;
    settings.invert_mask = DEFAULT_STEPPING_INVERT_MASK;
    settings.junction_deviation = DEFAULT_JUNCTION_DEVIATION;
  }
  // New settings since last version
  settings.flags = 0;
  if (DEFAULT_AUTO_START) { settings.flags |= BITFLAG_AUTO_START; }
  if (DEFAULT_HARD_LIMIT_ENABLE) { settings.flags |= BITFLAG_HARD_LIMIT_ENABLE; }
  if (DEFAULT_HOMING_ENABLE) { settings.flags |= BITFLAG_HOMING_ENABLE; }
  settings.homing_dir_mask = DEFAULT_HOMING_DIR_MASK;
  settings.homing_feed_rate = DEFAULT_HOMING_FEEDRATE;
  settings.homing_seek_rate = DEFAULT_HOMING_RAPID_FEEDRATE;
  settings.homing_debounce_delay = DEFAULT_HOMING_DEBOUNCE_DELAY;
  settings.homing_pulloff = DEFAULT_HOMING_PULLOFF;
  settings.stepper_idle_lock_time = DEFAULT_STEPPER_IDLE_LOCK_TIME;
  settings.decimal_places = DEFAULT_DECIMAL_PLACES;
}

// static void settings_startup_string(char *buf) {
//   memcpy_from_eeprom_with_checksum((char*)buf,512, 4);
// }

void settings_dump() {
  printPgmString(PSTR("$0 = ")); printFloat(settings.steps_per_mm[X_AXIS]);
  printPgmString(PSTR(" (x axis, steps/mm)\r\n$1 = ")); printFloat(settings.steps_per_mm[Y_AXIS]);
  printPgmString(PSTR(" (y axis, steps/mm)\r\n$2 = ")); printFloat(settings.steps_per_mm[Z_AXIS]);
  printPgmString(PSTR(" (z axis, steps/mm)\r\n$3 = ")); printInteger(settings.pulse_microseconds);
  printPgmString(PSTR(" (step pulse, usec)\r\n$4 = ")); printFloat(settings.default_feed_rate);
  printPgmString(PSTR(" (default feed rate, mm/min)\r\n$5 = ")); printFloat(settings.default_seek_rate);
  printPgmString(PSTR(" (default seek rate, mm/min)\r\n$6 = ")); printFloat(settings.mm_per_arc_segment);
  printPgmString(PSTR(" (arc resolution, mm/segment)\r\n$7 = ")); printInteger(settings.invert_mask); 
  printPgmString(PSTR(" (step port invert mask, int:binary = ")); print_uint8_base2(settings.invert_mask);  
  printPgmString(PSTR(")\r\n$8 = ")); printFloat(settings.acceleration/(60*60)); // Convert from mm/min^2 for human readability
  printPgmString(PSTR(" (acceleration, mm/sec^2)\r\n$9 = ")); printFloat(settings.junction_deviation);
  printPgmString(PSTR(" (cornering junction deviation, mm)\r\n$10 = ")); printInteger(bit_istrue(settings.flags,BITFLAG_REPORT_INCHES));
  printPgmString(PSTR(" (status report inches, bool)\r\n$11 = ")); printInteger(bit_istrue(settings.flags,BITFLAG_AUTO_START));
  printPgmString(PSTR(" (auto start enable, bool)\r\n$12 = ")); printInteger(bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE));
  printPgmString(PSTR(" (hard limit enable, bool)\r\n$13 = ")); printInteger(bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE));
  printPgmString(PSTR(" (homing enable, bool)\r\n$14 = ")); printInteger(settings.homing_dir_mask);
  printPgmString(PSTR(" (homing direction mask, int:binary = ")); print_uint8_base2(settings.homing_dir_mask);  
  printPgmString(PSTR(")\r\n$15 = ")); printFloat(settings.homing_feed_rate);
  printPgmString(PSTR(" (homing feed rate, mm/min)\r\n$16 = ")); printFloat(settings.homing_seek_rate);
  printPgmString(PSTR(" (homing seek rate, mm/min)\r\n$17 = ")); printInteger(settings.homing_debounce_delay);
  printPgmString(PSTR(" (homing debounce delay, msec)\r\n$18 = ")); printFloat(settings.homing_pulloff);
  printPgmString(PSTR(" (homing pull-off travel, mm)\r\n$19 = ")); printInteger(settings.stepper_idle_lock_time);
  printPgmString(PSTR(" (stepper idle lock time, msec)\r\n$20 = ")); printInteger(settings.decimal_places);
  printPgmString(PSTR(" (decimal places, int)")); 
  
//   char buf[4];
//   settings_startup_string((char *)buf);
//   printPgmString(PSTR("\r\n Startup: ")); printString(buf);
  
  printPgmString(PSTR("\r\n'$x=value' to set parameter or just '$' to dump current settings\r\n"));
}

// Parameter lines are on the form '$4=374.3' or '$' to dump current settings
uint8_t settings_execute_line(char *line) {
  uint8_t char_counter = 1;
//  unsigned char letter;
  float parameter, value;
  if(line[0] != '$') { 
    return(STATUS_UNSUPPORTED_STATEMENT); 
  }
  if(line[char_counter] == 0) { 
    settings_dump(); return(STATUS_OK); 
  }
//   if(line[char_counter] >= 'A' || line[char_counter] <= 'Z') {
//     letter = line[char_counter++];
//     if(line[char_counter++] != '=') { 
//       return(STATUS_UNSUPPORTED_STATEMENT); 
//     }
//     for (char_counter = 0; char_counter < LINE_BUFFER_SIZE-3; char_counter++) {
//       line[char_counter] = line[char_counter+3];
//     }
//     uint8_t status = gc_execute_line(line);
//     if (status) { return(status); }
//     else { settings_store_startup_line(line); }
//     
//     
//     // Opt stop and block delete are referred to as switches.
//     // How to store home position and work offsets real-time??
//     
//   } else {
  if(!read_float(line, &char_counter, &parameter)) {
    return(STATUS_BAD_NUMBER_FORMAT);
  };
  if(line[char_counter++] != '=') { 
    return(STATUS_UNSUPPORTED_STATEMENT); 
  }
  if(!read_float(line, &char_counter, &value)) {
    return(STATUS_BAD_NUMBER_FORMAT);
  }
  if(line[char_counter] != 0) { 
    return(STATUS_UNSUPPORTED_STATEMENT); 
  }
  settings_store_setting(parameter, value);
  return(STATUS_OK);
//   }
}

void write_settings() {
  eeprom_put_char(0, SETTINGS_VERSION);
  memcpy_to_eeprom_with_checksum(1, (char*)&settings, sizeof(settings_t));
//   
//   char buf[4]; buf[0] = 'G'; buf[1] = '2'; buf[2] = '0'; buf[3] = 0;
//   memcpy_to_eeprom_with_checksum(512, (char*)buf, 4);
//   
}

int read_settings() {
  // Check version-byte of eeprom
  uint8_t version = eeprom_get_char(0);
  
  if (version == SETTINGS_VERSION) {
    // Read settings-record and check checksum
    if (!(memcpy_from_eeprom_with_checksum((char*)&settings, 1, sizeof(settings_t)))) {
      return(false);
    }
  } else {
    // Incrementally update the old versions until up-to-date.
    if (version == 1) {
      // Migrate from settings version 1 to version 4.
      if (!(memcpy_from_eeprom_with_checksum((char*)&settings, 1, sizeof(settings_v1_t)))) {
        return(false);
      }
      settings.acceleration = DEFAULT_ACCELERATION;
      settings.junction_deviation = DEFAULT_JUNCTION_DEVIATION;
    } else if ((version == 2) || (version == 3)) {
      // Migrate from settings version 2 and 3 to version 4.
      if (!(memcpy_from_eeprom_with_checksum((char*)&settings, 1, sizeof(settings_v2_v4_t)))) {
        return(false);
      }
      if (version == 2) { settings.junction_deviation = DEFAULT_JUNCTION_DEVIATION; }    
      settings.acceleration *= 3600; // Convert to mm/min^2 from mm/sec^2
    } 
    if (version <= 4) {
      // Migrate from settings version 4 to current version.
      if (!(memcpy_from_eeprom_with_checksum((char*)&settings, 1, sizeof(settings_v2_v4_t)))) {
        return(false);
      }     
      settings_reset(false);        
      write_settings();
    } else if (version >= 50) {
      // Developmental settings. Version numbers greater than or equal to 50 are temporary.
      // Currently, this will update the user settings to v4 and the remainder of the settings
      // should be re-written to the default value, if the developmental version number changed.
      
      // Grab settings regardless of error.
      memcpy_from_eeprom_with_checksum((char*)&settings, 1, sizeof(settings_t));
      settings_reset(false);
      write_settings();
    } else {      
      return(false);
    }
  }
  return(true);
}

// A helper method to set settings from command line
void settings_store_setting(int parameter, float value) {
  switch(parameter) {
    case 0: case 1: case 2:
    if (value <= 0.0) {
      printPgmString(PSTR("Steps/mm must be > 0.0\r\n"));
      return;
    }
    settings.steps_per_mm[parameter] = value; break;
    case 3: 
    if (value < 3) {
      printPgmString(PSTR("Step pulse must be >= 3 microseconds\r\n"));
      return;
    }
    settings.pulse_microseconds = round(value); break;
    case 4: settings.default_feed_rate = value; break;
    case 5: settings.default_seek_rate = value; break;
    case 6: settings.mm_per_arc_segment = value; break;
    case 7: settings.invert_mask = trunc(value); break;
    case 8: settings.acceleration = value*60*60; break; // Convert to mm/min^2 for grbl internal use.
    case 9: settings.junction_deviation = fabs(value); break;
    case 10:
      if (value) { 
        settings.flags |= BITFLAG_REPORT_INCHES; 
      } else { settings.flags &= ~BITFLAG_REPORT_INCHES; }
      break;
    case 11:
      if (value) { 
        settings.flags |= BITFLAG_AUTO_START; 
      } else { settings.flags &= ~BITFLAG_AUTO_START; }
      break;
    case 12:
      if (value) { 
        settings.flags |= BITFLAG_HARD_LIMIT_ENABLE; 
      } else { settings.flags &= ~BITFLAG_HARD_LIMIT_ENABLE; }
      break;
    case 13:
      if (value) { 
        settings.flags |= BITFLAG_HOMING_ENABLE; 
        printPgmString(PSTR("Install all axes limit switches before use\r\n")); 
      } else { settings.flags &= ~BITFLAG_HOMING_ENABLE; }
      break;
    case 14: settings.homing_dir_mask = trunc(value); break;
    case 15: settings.homing_feed_rate = value; break;
    case 16: settings.homing_seek_rate = value; break;
    case 17: settings.homing_debounce_delay = round(value); break;
    case 18: settings.homing_pulloff = value; break;
    case 19:
       settings.stepper_idle_lock_time = round(value);
       // TODO: Immediately check and toggle steppers from always enable or disable?
       break;
    case 20: settings.decimal_places = round(value); break;
    default: 
      printPgmString(PSTR("Unknown parameter\r\n"));
      return;
  }
  write_settings();
  printPgmString(PSTR("Stored new setting\r\n"));
}

// Initialize the config subsystem
void settings_init() {
  if(!read_settings()) {
    printPgmString(PSTR("Warning: Failed to read EEPROM settings. Using defaults.\r\n"));
    settings_reset(true);
    write_settings();
    settings_dump();
  }  
}

// int8_t settings_execute_startup() {
// 
//   char buf[4];
//   settings_startup_string((char *)buf);
//   uint8_t i = 0;
//   while (i < 4) {
//     serial_write(buf[i++]);
//   }
//   return(gc_execute_line(buf));
// }
