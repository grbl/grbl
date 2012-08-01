/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2012 Sungeun K. Jeon

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

#include "spindle_control.h"
#include "settings.h"
#include "motion_control.h"
#include "config.h"
#include "planner.h"
#include "stepper.h"

#include <stdint.h>

static uint8_t current_direction;

void spindle_init()
{
  current_direction = 0;
  // SPINDLE_ENABLE and SPINDLE_DIRECTION set as outputs by SETUP_IO() in
  // config.h called from main()
  spindle_stop();
}

void spindle_stop()
{
  SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
}

void spindle_run(int direction, uint32_t rpm) 
{
  if (direction != current_direction) {
    plan_synchronize();
    if (direction) {
      #ifdef SPINDLE_DIRECTION
        if(direction > 0) {
          SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
        } else {
          SPINDLE_DIRECTION_PORT |= 1<<SPINDLE_DIRECTION_BIT;
        }
      #endif
      SPINDLE_ENABLE_PORT |= 1<<SPINDLE_ENABLE_BIT;
    } else {
      spindle_stop();
    }
    current_direction = direction;
  }
}
