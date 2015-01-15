/*
  grbl.h - include file
  Part of Grbl v0.9

  Copyright (c) 2015 Sungeun K. Jeon

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

// NOTE: This is not used by the 'make' compiling method. This is currently only used for
// simplifying compiling through the Arduino IDE at the moment. However, it may eventually
// turn into a central include file for the overall system.

#ifndef grbl_h
#define grbl_h

// All of the Grbl system include files.
#include "config.h"
#include "coolant_control.h"
#include "cpu_map.h"
#include "defaults.h"
#include "eeprom.h"
#include "gcode.h"
#include "limits.h"
#include "motion_control.h"
#include "nuts_bolts.h"
#include "planner.h"
#include "print.h"
#include "probe.h"
#include "protocol.h"
#include "report.h"
#include "serial.h"
#include "settings.h"
#include "spindle_control.h"
#include "stepper.h"
#include "system.h"

#endif
