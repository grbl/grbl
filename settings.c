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
#include "report.h"

settings_t settings;

// Version 4 outdated settings record
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
} settings_v4_t;

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
#define DEFAULT_INVERT_ST_ENABLE 0 // false
#define DEFAULT_HARD_LIMIT_ENABLE 0  // false
#define DEFAULT_HOMING_ENABLE 0  // false
#define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
#define DEFAULT_HOMING_RAPID_FEEDRATE 250.0 // mm/min
#define DEFAULT_HOMING_FEEDRATE 50 // mm/min
#define DEFAULT_HOMING_DEBOUNCE_DELAY 100 // msec (0-65k)
#define DEFAULT_HOMING_PULLOFF 1 // mm
#define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-255)
#define DEFAULT_DECIMAL_PLACES 3
#define DEFAULT_N_ARC_CORRECTION 25


void settings_write_coord_data(uint8_t coord_select, float *coord_data)
{  
  uint16_t addr = coord_select*(sizeof(float)*N_AXIS+1) + EEPROM_ADDR_PARAMETERS;
  memcpy_to_eeprom_with_checksum(addr,(char*)coord_data, sizeof(float)*N_AXIS);
}  


void write_global_settings() 
{
  eeprom_put_char(0, SETTINGS_VERSION);
  memcpy_to_eeprom_with_checksum(EEPROM_ADDR_GLOBAL, (char*)&settings, sizeof(settings_t));
}


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
  if (DEFAULT_REPORT_INCHES) { settings.flags |= BITFLAG_REPORT_INCHES; }
  if (DEFAULT_AUTO_START) { settings.flags |= BITFLAG_AUTO_START; }
  if (DEFAULT_INVERT_ST_ENABLE) { settings.flags |= BITFLAG_INVERT_ST_ENABLE; }
  if (DEFAULT_HARD_LIMIT_ENABLE) { settings.flags |= BITFLAG_HARD_LIMIT_ENABLE; }
  if (DEFAULT_HOMING_ENABLE) { settings.flags |= BITFLAG_HOMING_ENABLE; }
  settings.homing_dir_mask = DEFAULT_HOMING_DIR_MASK;
  settings.homing_feed_rate = DEFAULT_HOMING_FEEDRATE;
  settings.homing_seek_rate = DEFAULT_HOMING_RAPID_FEEDRATE;
  settings.homing_debounce_delay = DEFAULT_HOMING_DEBOUNCE_DELAY;
  settings.homing_pulloff = DEFAULT_HOMING_PULLOFF;
  settings.stepper_idle_lock_time = DEFAULT_STEPPER_IDLE_LOCK_TIME;
  settings.decimal_places = DEFAULT_DECIMAL_PLACES;
  settings.n_arc_correction = DEFAULT_N_ARC_CORRECTION;
  write_global_settings();
  
  // Zero all coordinate parameter data.
  float coord_data[N_AXIS];
  clear_vector_float(coord_data);
  uint8_t i = 0;
  for (i=0; i<=SETTING_INDEX_NCOORD; i++) {
    settings_write_coord_data(i,coord_data);
  }
}

// Read selected coordinate data from EEPROM. Updates pointed coord_data value.
uint8_t settings_read_coord_data(uint8_t coord_select, float *coord_data)
{
  uint16_t addr = coord_select*(sizeof(float)*N_AXIS+1) + EEPROM_ADDR_PARAMETERS;
  if ((!memcpy_from_eeprom_with_checksum((char*)coord_data, addr, sizeof(float)*N_AXIS))) {
    clear_vector_float(coord_data);  // Default zero vector
    settings_write_coord_data(coord_select,coord_data);
    return(false);
  } else {
    return(true);
  }
}  


uint8_t read_global_settings() {
  // Check version-byte of eeprom
  uint8_t version = eeprom_get_char(0);
  
  if (version == SETTINGS_VERSION) {
    // Read settings-record and check checksum
    if (!(memcpy_from_eeprom_with_checksum((char*)&settings, EEPROM_ADDR_GLOBAL, sizeof(settings_t)))) {
      return(false);
    }
  } else {
    if (version <= 4) {
      // Migrate from settings version 4 to current version.
      if (!(memcpy_from_eeprom_with_checksum((char*)&settings, 1, sizeof(settings_v4_t)))) {
        return(false);
      }     
      settings_reset(false);        
    } else if (version >= 50) {
      // Developmental settings. Version numbers greater than or equal to 50 are temporary.
      // Currently, this will update the user settings to v4 and the remainder of the settings
      // should be re-written to the default value, if the developmental version number changed.
      
      // Grab settings regardless of error.
      memcpy_from_eeprom_with_checksum((char*)&settings, 1, sizeof(settings_t));
      settings_reset(false);
    } else {      
      return(false);
    }
  }
  return(true);
}


// A helper method to set settings from command line
uint8_t settings_store_global_setting(int parameter, float value) {
  switch(parameter) {
    case 0: case 1: case 2:
      if (value <= 0.0) { return(STATUS_SETTING_VALUE_NEG); } 
      settings.steps_per_mm[parameter] = value; break;
    case 3: 
      if (value < 3) { return(STATUS_SETTING_STEP_PULSE_MIN); }
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
        settings.flags |= BITFLAG_INVERT_ST_ENABLE; 
      } else { settings.flags &= ~BITFLAG_INVERT_ST_ENABLE; }
      break;
    case 13:
      if (value) { 
        settings.flags |= BITFLAG_HARD_LIMIT_ENABLE; 
      } else { settings.flags &= ~BITFLAG_HARD_LIMIT_ENABLE; }
      break;
    case 14:
      if (value) { 
        settings.flags |= BITFLAG_HOMING_ENABLE; 
        report_feedback_message(MESSAGE_HOMING_ENABLE);
      } else { settings.flags &= ~BITFLAG_HOMING_ENABLE; }
      break;
    case 15: settings.homing_dir_mask = trunc(value); break;
    case 16: settings.homing_feed_rate = value; break;
    case 17: settings.homing_seek_rate = value; break;
    case 18: settings.homing_debounce_delay = round(value); break;
    case 19: 
      if (value <= 0.0) { return(STATUS_SETTING_VALUE_NEG); } 
      settings.homing_pulloff = value; break;
    case 20:
       settings.stepper_idle_lock_time = round(value);
       // TODO: Immediately check and toggle steppers from always enable or disable?
       break;
    case 21: settings.decimal_places = round(value); break;
    case 22: settings.n_arc_correction = round(value); break;
    default: 
      return(STATUS_INVALID_STATEMENT);
  }
  write_global_settings();
  return(STATUS_OK);
}

// Initialize the config subsystem
void settings_init() {
  if(!read_global_settings()) {
    report_status_message(STATUS_SETTING_READ_FAIL);
    settings_reset(true);
    report_grbl_help();
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
