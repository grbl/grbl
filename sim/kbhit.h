/*
  kbhit.h - keyboard hit detection - used in serial port replacement for grbl sim

  linux kbhit taken from http://cboard.cprogramming.com/c-programming/63166-kbhit-linux.html. 
  By 'thanatos':http://cboard.cprogramming.com/member.php?u=380

  Part of Grbl Simulator

  Copyright (c) 2014 Adam Shelly

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

//if linux
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

int  kbhit(void);
void enable_kbhit(int);


//else
//#include <conio.h>
//#define enable_kbhit(e) 
//#define kbhit _kbhit
//endif

