/*
  defaults_pulpitrockcnc.h - Grbl settings for the PulpitRockCNC machine based RAMPS1.4
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

/* The defaults.h file serves as a central default settings file for different machine
   types, from DIY CNC mills to CNC conversions of off-the-shelf machines. The settings
   here are supplied by users, so your results may vary. However, this should give you
   a good starting point as you get to know your machine and tweak the settings for your
   nefarious needs. */

#ifndef defaults_h
#define defaults_h

  // Grbl settings for the PulpitRockCNC
  #define MICROSTEPS 16 // 16 --> all three jumpers installed
  #define STEPS_PER_REV 200.0
  #define MM_PER_REV 1.25 // 1.25 mm/rev leadscrew
  #define DEFAULT_X_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/MM_PER_REV)
  #define DEFAULT_Y_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/MM_PER_REV)
  #define DEFAULT_Z_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/MM_PER_REV)
  #define DEFAULT_X_MAX_RATE 7*60.0 // mm/min, cyclone:5, 7 is max for Pulpit Rock CNC
  #define DEFAULT_Y_MAX_RATE 7*60.0 // mm/min, cyclone:5, 7 is max for Pulpit Rock CNC
  #define DEFAULT_Z_MAX_RATE 7*60.0 // mm/min, cyclone:2.5, 7 is max for Pulpit Rock CNC
  #define DEFAULT_X_ACCELERATION (16.0*60*60) // 50*60*60 mm/min^2 = 50 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (16.0*60*60) // 50*60*60 mm/min^2 = 50 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (16.0*60*60) // 50*60*60 mm/min^2 = 50 mm/sec^2
  // The Pulpit Rock CNC has the following dimensions:
  // X Min = 0
  // X Max = 365
  // Y Min = 0
  // Y Max = 234
  // Z Min = 0
  // Z Max = 123
  #define DEFAULT_X_MAX_TRAVEL 360.0 // mm 360.0
  #define DEFAULT_Y_MAX_TRAVEL 230.0 // mm 230.0
  #define DEFAULT_Z_MAX_TRAVEL 134.0 // mm (from 0 to -135) 134.0
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((0<<X_AXIS)|(0<<Y_AXIS)|(0<<Z_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled), cyclone:255
  #define DEFAULT_STATUS_REPORT_MASK ((BITFLAG_RT_STATUS_MACHINE_POSITION)|(BITFLAG_RT_STATUS_WORK_POSITION)) // |(BITFLAG_RT_STATUS_LIMIT_PINS) only used for debugging limit switches
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_AUTO_START 1 // true
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  // Uses software limits in the firmware to keep the printer from going too far in the opposite direction and hardware endstops for min
  #define DEFAULT_SOFT_LIMIT_ENABLE 1 // true
  #define DEFAULT_HARD_LIMIT_ENABLE 1 // true
  #define DEFAULT_HOMING_ENABLE 1 // true
  #define DEFAULT_HOMING_DIR_MASK ((1<<X_AXIS)|(1<<Y_AXIS)|(0<<Z_AXIS)) // X and Y endstop installed to the minimum (zero point). Z max installed and Z Min used for probe
  #define DEFAULT_HOMING_FEED_RATE 50.0 // mm/min (slower feed rate to "bump" the endstops)
  #define DEFAULT_HOMING_SEEK_RATE 1000.0 // mm/min (will saturate to MAX_RATE)
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm (distance that the axis move after homing) 

#endif
