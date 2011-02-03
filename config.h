/*
  config.h - eeprom and compile time configuration handling 
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

#ifndef config_h
#define config_h

#define VERSION "0.6b"
#include <math.h>
#include <inttypes.h>

// Settings that can only be set at compile-time:

#define BAUD_RATE 9600
//#define BAUD_RATE 115200

// Default pin-assignments from Grbl 0.5

// #define STEPPERS_ENABLE_DDR     DDRD
// #define STEPPERS_ENABLE_PORT    PORTD
// #define STEPPERS_ENABLE_BIT         2
// 
// #define STEPPING_DDR       DDRC
// #define STEPPING_PORT      PORTC 
// #define X_STEP_BIT           0
// #define Y_STEP_BIT           1
// #define Z_STEP_BIT           2
// #define X_DIRECTION_BIT            3
// #define Y_DIRECTION_BIT            4
// #define Z_DIRECTION_BIT            5
// 
// #define LIMIT_DDR      DDRD
// #define LIMIT_PORT     PORTD
// #define X_LIMIT_BIT          3
// #define Y_LIMIT_BIT          4
// #define Z_LIMIT_BIT          5
// 
// #define SPINDLE_ENABLE_DDR DDRD
// #define SPINDLE_ENABLE_PORT PORTD
// #define SPINDLE_ENABLE_BIT 6
// 
// #define SPINDLE_DIRECTION_DDR DDRD
// #define SPINDLE_DIRECTION_PORT PORTD
// #define SPINDLE_DIRECTION_BIT 7
// 
// 


// Updated default pin-assignments from 0.6 onwards

#define STEPPERS_ENABLE_DDR     DDRB
#define STEPPERS_ENABLE_PORT    PORTB
#define STEPPERS_ENABLE_BIT         0

#define STEPPING_DDR       DDRD
#define STEPPING_PORT      PORTD
#define X_STEP_BIT           2
#define Y_STEP_BIT           3
#define Z_STEP_BIT           4
#define X_DIRECTION_BIT      5
#define Y_DIRECTION_BIT      6
#define Z_DIRECTION_BIT      7

#define LIMIT_DDR      DDRB
#define LIMIT_PORT     PORTB
#define X_LIMIT_BIT          1
#define Y_LIMIT_BIT          2
#define Z_LIMIT_BIT          3

#define SPINDLE_ENABLE_DDR DDRB
#define SPINDLE_ENABLE_PORT PORTB
#define SPINDLE_ENABLE_BIT 4

#define SPINDLE_DIRECTION_DDR DDRB
#define SPINDLE_DIRECTION_PORT PORTB
#define SPINDLE_DIRECTION_BIT 5


// Version of the EEPROM data. Will be used to migrate existing data from older versions of Grbl
// when firmware is upgraded. Always stored in byte 0 of eeprom
#define SETTINGS_VERSION 2

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
  double max_jerk;
} settings_t;
extern settings_t settings;

// Initialize the configuration subsystem (load settings from EEPROM)
void config_init();

// Print current settings
void dump_settings();

// A helper method to set new settings from command line
void store_setting(int parameter, double value);

// Default settings (used when resetting eeprom-settings)
#define MICROSTEPS 8
#define X_STEPS_PER_MM (94.488188976378*MICROSTEPS)
#define Y_STEPS_PER_MM (94.488188976378*MICROSTEPS)
#define Z_STEPS_PER_MM (94.488188976378*MICROSTEPS)
#define STEP_PULSE_MICROSECONDS 30
#define MM_PER_ARC_SEGMENT 0.1
#define RAPID_FEEDRATE 480.0 // in millimeters per minute
#define DEFAULT_FEEDRATE 480.0
#define DEFAULT_ACCELERATION (DEFAULT_FEEDRATE/100.0)
#define DEFAULT_MAX_JERK 50.0
#define DEFAULT_STEPPING_INVERT_MASK 0

// The temporal resolution of the acceleration management subsystem
#define ACCELERATION_TICKS_PER_SECOND 20L

#endif
