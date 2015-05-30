/*
  defaults_ballscrew.h - defaults settings configuration file
  Part of Grbl

  Copyright (c) 2012-2015 Sungeun K. Jeon

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

/*
  Theses defaults are set up for 1605 based ballscrew systems driven by NEMA 23
  type motors on large format tables.
*/

#ifndef defaults_h
#define defaults_h
  #define RESET_SETTINGS 1
  #undef  SPINDLE_MAX_RPM
  #define SPINDLE_MAX_RPM 24000.0
  #undef  SPINDLE_MIN_RPM
  #define SPINDLE_MIN_RPM 0.0
  #define ENABLE_M7
  #define MICROSTEPS_XY 4
  #define MAX_SPEED 250.0
  #define STEP_REVS_XY 200
  #define MM_PER_REV_XY (5)
  #define MICROSTEPS_Z 4
  #define STEP_REVS_Z 200
  #define MM_PER_REV_Z (5)
  #define DEFAULT_X_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Y_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Z_STEPS_PER_MM (MICROSTEPS_Z*STEP_REVS_Z/MM_PER_REV_Z)
  #define DEFAULT_X_MAX_RATE MAX_SPEED // mm/min
  #define DEFAULT_Y_MAX_RATE MAX_SPEED
  #define DEFAULT_Z_MAX_RATE MAX_SPEED
  #define DEFAULT_X_ACCELERATION (10.0*60*60)
  #define DEFAULT_Y_ACCELERATION (10.0*60*60)
  #define DEFAULT_Z_ACCELERATION (10.0*60*60)
  #define DEFAULT_X_MAX_TRAVEL 1200.0 // mm
  #define DEFAULT_Y_MAX_TRAVEL 2300.0 // mm
  #define DEFAULT_Z_MAX_TRAVEL 370.0 // mm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 20
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((0<<X_AXIS)|(0<<Z_AXIS)|(1<<Y_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 254 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 255 // All enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.01 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 1 // true
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE MAX_SPEED/2 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE MAX_SPEED
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 25 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm

#endif
