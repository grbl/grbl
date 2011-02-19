/*
  config.h - compile time configuration
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

#define BAUD_RATE 9600

// Updated default pin-assignments from 0.6 onwards 
// (see bottom of file for a copy of the old config)

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
#define LIMIT_PIN     PINB
#define X_LIMIT_BIT          1
#define Y_LIMIT_BIT          2
#define Z_LIMIT_BIT          3

#define SPINDLE_ENABLE_DDR DDRB
#define SPINDLE_ENABLE_PORT PORTB
#define SPINDLE_ENABLE_BIT 4

#define SPINDLE_DIRECTION_DDR DDRB
#define SPINDLE_DIRECTION_PORT PORTB
#define SPINDLE_DIRECTION_BIT 5

// The temporal resolution of the acceleration management subsystem. Higher number
// give smoother acceleration but may impact performance
#define ACCELERATION_TICKS_PER_SECOND 40L

#endif

// Pin-assignments from Grbl 0.5

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

