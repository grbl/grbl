/*
  config.h - configuration data for Grbl
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

#define VERSION "0.0"

#define X_STEPS_PER_MM 2.0
#define Y_STEPS_PER_MM 2.0
#define Z_STEPS_PER_MM 2.0

#define INCHES_PER_MM 25.4
#define X_STEPS_PER_INCH X_STEPS_PER_MM*INCHES_PER_MM
#define Y_STEPS_PER_INCH Y_STEPS_PER_MM*INCHES_PER_MM
#define Z_STEPS_PER_INCH Z_STEPS_PER_MM*INCHES_PER_MM

#define RAPID_FEEDRATE 200000.0 // in millimeters per minute
#define DEFAULT_FEEDRATE 200000.0

#define STEPPERS_ENABLE_DDR     DDRB
#define STEPPERS_ENABLE_PORT    PORTB
#define STEPPERS_ENABLE_BIT         6

#define STEPPING_DDR       DDRB
#define STEPPING_PORT      PORTB 
#define X_STEP_BIT           0
#define Y_STEP_BIT           1
#define Z_STEP_BIT           2
#define X_DIRECTION_BIT            3
#define Y_DIRECTION_BIT            4
#define Z_DIRECTION_BIT            5

#define LIMIT_DDR      DDRC
#define LIMIT_PORT     PORTC 
#define X_LIMIT_BIT          0
#define Y_LIMIT_BIT          1
#define Z_LIMIT_BIT          2

#define SPINDLE_ENABLE_DDR DDRC
#define SPINDLE_ENABLE_PORT PORTC
#define SPINDLE_ENABLE_BIT 3

#define SPINDLE_DIRECTION_DDR DDRC
#define SPINDLE_DIRECTION_PORT PORTC
#define SPINDLE_DIRECTION_BIT 4

#define BAUD_RATE 9600

#define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT))
#define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT))
#define STEPPING_MASK (STEP_MASK | DIRECTION_MASK)
#define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT))

#endif
