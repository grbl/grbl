/*
  defaults_cyclone2_1.h - GRBL settings for Cyclone PCB Factory v2.1
  http://reprap.org/wiki/Cyclone_PCB_Factory
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

  // Description: GRBL settings for Cyclone PCB Factory v2.1
  // http://reprap.org/wiki/Cyclone_PCB_Factory
  #define MICROSTEPS 16 // 16 --> all three jumpers installed
  #define STEPS_PER_REV 200.0
  #define MM_PER_REV 1.25 // 1.25 mm/rev leadscrew
  #define Cyclone_XY_Gear_Ratio 21.0/21.0 // Number of gear teeth (motor/rod)
  #define Cyclone_Z_Gear_Ratio 8.0/15.0 // Number of gear teeth (motor/rod)
  #define DEFAULT_X_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/(Cyclone_XY_Gear_Ratio*MM_PER_REV))
  #define DEFAULT_Y_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/(Cyclone_XY_Gear_Ratio*MM_PER_REV))
  #define DEFAULT_Z_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/(Cyclone_Z_Gear_Ratio*MM_PER_REV))
  #define DEFAULT_X_MAX_RATE 5*60.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 5*60.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 2.5*60.0 // mm/min
  #define DEFAULT_X_ACCELERATION (16.0*60*60) // 50*60*60 mm/min^2 = 50 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (16.0*60*60) // 50*60*60 mm/min^2 = 50 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (16.0*60*60) // 50*60*60 mm/min^2 = 50 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 168.0 // mm
  #define DEFAULT_Y_MAX_TRAVEL 101.0 // mm
  #define DEFAULT_Z_MAX_TRAVEL 50.0 // mm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((0<<X_AXIS)|(0<<Y_AXIS)|(1<<Z_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK ((BITFLAG_RT_STATUS_MACHINE_POSITION)|(BITFLAG_RT_STATUS_WORK_POSITION))
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_AUTO_START 1 // true
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 1 // true
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_HOMING_ENABLE 1  // true
  #define DEFAULT_HOMING_DIR_MASK ((1<<X_AXIS)|(1<<Y_AXIS)|(0<<Z_AXIS)) // in Cyclone, z axis is left to move upwards, in case Z homing is triggered (there is no Z endstop!)
  #define DEFAULT_HOMING_FEED_RATE 50.0 // mm/min (slower feed rate to "bump" the endstops)
  #define DEFAULT_HOMING_SEEK_RATE 635.0 // mm/min (will saturate to MAX_RATE)
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 0.0 // mm (distance that the axis move after homing)

#endif
