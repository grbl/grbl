/*
  spindle_control.c - spindle control methods
  Part of Grbl

  The MIT License (MIT)

  GRBL(tm) - Embedded CNC g-code interpreter and motion-controller
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2012 Sungeun K. Jeon

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

#include "settings.h"
#include "spindle_control.h"
#include "planner.h"

static uint8_t current_direction;

void spindle_init()
{
  current_direction = 0;
  SPINDLE_SPEED_DDR |= (1 << SPINDLE_ENABLE_BIT);		// OUTPUT
  SPINDLE_ENABLE_DDR |= (1 << SPINDLE_ENABLE_BIT);
  SPINDLE_DIRECTION_DDR |= (1 << SPINDLE_DIRECTION_BIT);  
  spindle_stop();
}

void spindle_stop()
{
	spindle_speed(0);
	SPINDLE_ENABLE_PORT &= ~(1 << SPINDLE_ENABLE_BIT);
}

void spindle_run(int8_t direction) //, uint16_t rpm) 
{
  if (direction != current_direction) {
    plan_synchronize();
    if (direction) {
      if(direction > 0) {
        SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
      } else {
        SPINDLE_DIRECTION_PORT |= 1<<SPINDLE_DIRECTION_BIT;
      }
      SPINDLE_ENABLE_PORT |= 1<<SPINDLE_ENABLE_BIT;
    } else {
      spindle_stop();     
    }
    current_direction = direction;
  }
}

void spindle_speed(uint8_t speed)		// Speed is analog 0 - 255 (0 to 5v)
{
#ifdef SPINDLE_SPEEDCONTROL_PIN11
	SPINDLE_SPEED_DDR |= (1 << SPINDLE_SPEED_BIT);	// OUTPUT mode on PIN 11
	if(speed == 0)
	{
		TCCR2A &= ~(1 << COM2A1);  // Output voltage is zero on PIN 11
		SPINDLE_SPEED_PORT &= ~(1 << SPINDLE_SPEED_BIT);
		return;
	}
	else if(speed == 255)
	{
		TCCR2A &= ~(1 << COM2A1);  // Output voltage is zero on PIN 11
		SPINDLE_SPEED_PORT |= (1 << SPINDLE_SPEED_BIT);		// Full on
		return;
	}
	//
	// Set speed PWM...
	TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
	TCCR2B = (TCCR2B & 0b11111000) | 0x02; // set to 1/8 Prescaler
	OCR2A = speed;
#endif
}
