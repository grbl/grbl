/*
  config.h - eeprom and compile time configuration handling 
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

#ifndef config_h
#define config_h

#define VERSION "0.60"

// Settings that can only be set at compile-time:

#ifndef BAUD_RATE
#define BAUD_RATE 115200
#endif

#define STEPPERS_ENABLE_DDR     DDRB		// pin 13, with LED
#define STEPPERS_ENABLE_PORT    PORTB
#define STEPPERS_ENABLE_BIT     5	

#define STEPPING_DDR            DDRD
#define STEPPING_PORT           PORTD 
#define X_STEP_BIT              7
#define Y_STEP_BIT              5
#define Z_STEP_BIT              3
#define X_DIRECTION_BIT         6
#define Y_DIRECTION_BIT         4
#define Z_DIRECTION_BIT         2

#define LIMIT_DDR               DDRC
#define LIMIT_PORT              PORTC
#define X_LIMIT_BIT             0
#define Y_LIMIT_BIT             1
#define Z_LIMIT_BIT             2

#define SPINDLE_ENABLE_DDR      DDRC
#define SPINDLE_ENABLE_PORT     PORTC
#define SPINDLE_ENABLE_BIT      3

#define SPINDLE_DIRECTION_DDR   DDRD
#define SPINDLE_DIRECTION_PORT  PORTD
#define SPINDLE_DIRECTION_BIT   2

#define LCD_DB0 	10				// Using Ardiuno numbering, not port numbering
#define LCD_DB1		11				// Equivalent to PORTD, pins 7 to 2
#define LCD_DB2		12
#define LCD_DB3		13
#define LCD_ENABLE	9
#define LCD_RS 		8

// Version of the EEPROM data. Will be used to migrate existing data from older versions of Grbl
// when firmware is upgraded. Always stored in byte 0 of eeprom
#define SETTINGS_VERSION 2

// Current global settings (persisted in EEPROM from byte 1 onwards)
struct Settings {
  double 	steps_per_mm[3];
  uint8_t 	microsteps;
  uint8_t 	pulse_microseconds;
  double 	default_feed_rate;
  double 	default_seek_rate;
  uint8_t 	invert_mask;
  double 	mm_per_arc_segment;
  uint32_t  backlash_x_count;
  uint32_t  backlash_y_count;
  uint32_t  backlash_z_count;
};
struct Settings settings;

// Initialize the configuration subsystem (load settings from EEPROM)
void config_init();
void config_reset();

// Print current settings
void dump_settings();

// A helper method to set new settings from command line
void store_setting(int parameter, double value);

// Default settings (used when resetting eeprom-settings)
#define MICROSTEPS 8
//#define DEFAULT_X_STEPS_PER_MM (94.488188976378*MICROSTEPS)
//#define DEFAULT_Y_STEPS_PER_MM (94.488188976378*MICROSTEPS)
//#define DEFAULT_Z_STEPS_PER_MM (94.488188976378*MICROSTEPS)

#define DEFAULT_X_STEPS_PER_MM (200/1.27*MICROSTEPS)
#define DEFAULT_Y_STEPS_PER_MM (DEFAULT_X_STEPS_PER_MM)
#define DEFAULT_Z_STEPS_PER_MM (DEFAULT_X_STEPS_PER_MM)

// UM per step is used in display routines. It uses um rather than
// mm, as the roundoff error works better, and the value is divided
// in the display routine back to mm.
#define DEFAULT_UM_PER_STEP (1000* 1.27/(200*MICROSTEPS))

#define DEFAULT_STEP_PULSE_MICROSECONDS 30
#define DEFAULT_MM_PER_ARC_SEGMENT 0.1
#define DEFAULT_SEEKRATE 480.0 // in millimeters per minute
#define DEFAULT_FEEDRATE 480.0

// Some useful constants
#define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT))                     // All step bits
#define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits
#define STEPPING_MASK (STEP_MASK | DIRECTION_MASK)                                      // All stepping-related bits (step/direction)
#define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT))                 // All limit bits

#endif
