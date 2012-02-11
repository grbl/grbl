/*
  spindle_control.c - spindle control methods
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

#include "spindle_control.h"
#include "settings.h"
#include "config.h"
#include "stepper.h"

#include <avr/io.h>

static int current_direction;

void spindle_init()
{
  SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT);
  SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT);
  spindle_run(0, 0);
}

void spindle_run(int direction, uint32_t rpm) 
{
  if (direction != current_direction) {
    st_synchronize();
    if(direction) {
      if(direction > 0) {
        SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
      } else {
        SPINDLE_DIRECTION_PORT |= 1<<SPINDLE_DIRECTION_BIT;
      }
      SPINDLE_ENABLE_PORT |= 1<<SPINDLE_ENABLE_BIT;
    } else {
      SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);      
    }
    current_direction = direction;
  }
}

void spindle_stop()
{
  spindle_run(0, 0);
}
