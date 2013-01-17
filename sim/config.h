/*
  config.h - replacement for the include of the same name in grbl
  to define dummy registers

  Part of Grbl Simulator

  Copyright (c) 2012 Jens Geisler

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
#include "../config.h"
#include <inttypes.h>


// dummy register variables implemented in simulator.c
extern uint8_t stepping_ddr;
extern uint8_t stepping_port;
extern uint8_t spindle_ddr;
extern uint8_t spindle_port;
extern uint8_t limit_ddr;
extern uint8_t limit_port;
extern uint8_t limit_int_reg;
extern uint8_t pinout_ddr;
extern uint8_t pinout_port;
extern uint8_t pinout_int_reg;
extern uint8_t coolant_flood_ddr;
extern uint8_t coolant_flood_port;

// ReDefine pin-assignments
#undef STEPPING_DDR
#define STEPPING_DDR       stepping_ddr
#undef STEPPING_PORT
#define STEPPING_PORT      stepping_port
#undef X_STEP_BIT
#define X_STEP_BIT         2  // Uno Digital Pin 2
#undef Y_STEP_BIT
#define Y_STEP_BIT         3  // Uno Digital Pin 3
#undef Z_STEP_BIT
#define Z_STEP_BIT         4  // Uno Digital Pin 4
#undef X_DIRECTION_BIT
#define X_DIRECTION_BIT    5  // Uno Digital Pin 5
#undef Y_DIRECTION_BIT
#define Y_DIRECTION_BIT    6  // Uno Digital Pin 6
#undef Z_DIRECTION_BIT
#define Z_DIRECTION_BIT    7  // Uno Digital Pin 7

#undef STEPPERS_DISABLE_DDR
#define STEPPERS_DISABLE_DDR    stepping_ddr
#undef STEPPERS_DISABLE_PORT
#define STEPPERS_DISABLE_PORT   stepping_port
#undef STEPPERS_DISABLE_BIT
#define STEPPERS_DISABLE_BIT    0  // Uno Digital Pin 8

#undef LIMIT_DDR
#define LIMIT_DDR     limit_ddr
#undef LIMIT_PORT
#define LIMIT_PORT     limit_port
#undef LIMIT_PIN
#define LIMIT_PIN     limit_port
#undef X_LIMIT_BIT
#define X_LIMIT_BIT   1  // Uno Digital Pin 9
#undef Y_LIMIT_BIT
#define Y_LIMIT_BIT   2  // Uno Digital Pin 10
#undef Z_LIMIT_BIT
#define Z_LIMIT_BIT   3  // Uno Digital Pin 11
#undef LIMIT_INT
#define LIMIT_INT 0
#undef LIMIT_PCMSK
#define LIMIT_PCMSK limit_int_reg

#undef SPINDLE_ENABLE_DDR
#define SPINDLE_ENABLE_DDR spindle_ddr
#undef SPINDLE_ENABLE_PORT
#define SPINDLE_ENABLE_PORT spindle_port
#undef SPINDLE_ENABLE_BIT
#define SPINDLE_ENABLE_BIT 4  // Uno Digital Pin 12

#undef SPINDLE_DIRECTION_DDR
#define SPINDLE_DIRECTION_DDR spindle_ddr
#undef SPINDLE_DIRECTION_PORT
#define SPINDLE_DIRECTION_PORT spindle_port
#undef SPINDLE_DIRECTION_BIT
#define SPINDLE_DIRECTION_BIT 5  // Uno Digital Pin 13

#undef PINOUT_DDR
#define PINOUT_DDR pinout_ddr
#undef PINOUT_PORT
#define PINOUT_PORT pinout_port
#undef PINOUT_PIN
#define PINOUT_PIN pinout_port
#undef PINOUT_PCMSK
#define PINOUT_PCMSK pinout_int_reg
#undef PINOUT_INT
#define PINOUT_INT 0

#undef COOLANT_FLOOD_DDR
#define COOLANT_FLOOD_DDR coolant_flood_ddr
#undef COOLANT_FLOOD_PORT
#define COOLANT_FLOOD_PORT coolant_flood_port
#undef COOLANT_FLOOD_BIT
#define COOLANT_FLOOD_BIT 0


#endif
