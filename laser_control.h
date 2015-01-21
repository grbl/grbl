/*
  laser_control.h - laser control methods
  Part of Horus Firmware

  Copyright (c) 2014-2015 Jesus Arroyo (Mundo Reader S.L.)

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

#ifndef laser_control_h
#define laser_control_h 


void laser_init();
void laser_off(uint8_t laser_bit);
void laser_on(uint8_t laser_bit);
void laser_run(uint8_t mode, uint8_t value);

#endif