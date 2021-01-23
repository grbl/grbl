/*
  relay_control.h - relay control methods
  Part of Grbl

  Copyright (c) 2016 Puskin A. Andrei

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

#ifndef relay_control_h
#define relay_control_h 

void relay_init();
void relay_stop();
void relay_set_state(uint8_t mode);
void relay_run(uint8_t mode);

#endif
