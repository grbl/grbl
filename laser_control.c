/*
  laser.c - laser control methods
  Part of Horus Project

  Copyright (c) 2014 Jesus Arroyo (Mundo Reader S.L.)

  Horus is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Horus is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Horus.  If not, see <http://www.gnu.org/licenses/>.
*/  

#include "system.h"
#include "laser_control.h"
#include "protocol.h"
#include "gcode.h"


void laser_init()
{
  LASER_DDR |= LASER_MASK;
  LASER_PORT &= ~LASER_MASK;
}

void laser_off(uint8_t laser_bit)
{
  LASER_PORT &= ~laser_bit;
}

void laser_on(uint8_t laser_bit)
{
  LASER_PORT |= laser_bit;
}

void laser_run(uint8_t mode, uint8_t value)
{
  if (sys.state == STATE_CHECK_MODE) { return; }

  protocol_auto_cycle_start();
  protocol_buffer_synchronize();

  uint8_t bit = 0;

  switch (value) {
    case 1: bit = (1<<LASER1_BIT); break;
    case 2: bit = (1<<LASER2_BIT); break;
    case 3: bit = (1<<LASER3_BIT); break;
    case 4: bit = (1<<LASER4_BIT); break;
  }
  
  if (bit > 0) {
    if (mode == LASER_ENABLE) {
      laser_on(bit);
    } else {
      laser_off(bit);
    }
  }
}