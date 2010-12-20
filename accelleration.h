/*
  accelleration.h - accelleration management support 
  Part of Grbl

  Copyright (c) 2009 Simen Svale Skogsrud

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

#ifndef accelleration_h
#define accelleration_h 

// Unless someone else defined AC_TICKS_PER_SECOND, we define a sensible default
#ifndef AC_TICKS_PER_SECOND
#define AC_TICKS_PER_SECOND 10
#endif

struct AccellerationProfile {
  float initial_scaler;
  float final_scaler;
  float accelleration_delta;
  float decelleration_delta;
  uint32_t accellerate_ticks;
  uint32_t plateau_ticks;    
};

struct AccellerationProfileSegment {
  float v_entry[3];
  float v_ideal[3];
  float v_exit[3];
  float distance;
  float f_entry, f_exit;
};

struct AccellerationProfileBuilder {
  AccellerationProfileSegment segment[3];
  uint8_t current;
};

#endif 