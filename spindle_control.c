/*
  spindle_control.c - spindle control methods
  Part of Grbl

  The MIT License (MIT)

  Grbl(tm) - Embedded CNC g-code interpreter and motion-controller
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#include "spindle_control.h"
#include "settings.h"
#include "config.h"

#include <avr/io.h>

void spindle_init()
{
  SPINDLE_ENABLE_DDR |= 1<<SPINDLE_ENABLE_BIT;
}

void spindle_run(int direction, uint32_t rpm) 
{
  if(direction >= 0) {
    SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
  } else {
    SPINDLE_DIRECTION_PORT |= 1<<SPINDLE_DIRECTION_BIT;
  }
  SPINDLE_ENABLE_PORT |= 1<<SPINDLE_ENABLE_BIT;
}

void spindle_stop()
{
  SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
}
