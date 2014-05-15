/*
  config.h - compile time configuration
  Part of Grbl

  The MIT License (MIT)

  Grbl(tm) - Embedded CNC g-code interpreter and motion-controller
  Copyright (c) 2009 Simen Svale Skogsrud

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
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

