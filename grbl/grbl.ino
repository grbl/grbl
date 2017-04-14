// GRBL RAMPS 1.4 port
// Originally by ArSi arsi@arsi.sk
// Enhanced to support both Arduino Uno CPU and Arduino Mega 2560 + RAMPS 1.4 Board by Per Ivar Nerseth perivar@nerseth.com

// Copyright (c) 2009-2011 Simen Svale Skogsrud
// Copyright (c) 2012-2015 Sungeun K. Jeon

// Grbl is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// Grbl is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

#include "system.h"
#include "nuts_bolts.h"
#include "eeprom.h"
#include "gcode.h"

#include "config.h"
#include "defaults.h"
#include "settings.h"
#include "fastio.h"

#include "stepper.h"
#include "planner.h"
#include "report.h"
#include "serial.h"
#include "cpu_map.h"
#include "limits.h"
#include "motion_control.h"
#include "spindle_control.h"
#include "protocol.h"
#include "probe.h"
#include "print.h"
#include "coolant_control.h"






