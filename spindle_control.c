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
#include "config.h"
#include "settings.h"
#include "spindle_control.h"
#include "planner.h"
#include <avr/io.h>
#include <avr/interrupt.h>




//Cycle 50Hz
#define MAX F_CPU/8/50
#define MIN F_CPU/8/1429

static uint8_t current_direction;

void spindle_init()
{
  current_direction = 0;
  SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT);
  SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT);  

  SPINDLE_PWM_DDR 		|= (1<<SPINDLE_PWM_BIT);

  TCCR1A = (1<<COM1A1) | (1<<WGM11); // FAST PWM Mode 14
  TCCR1B = (1<<WGM13)  | (1<<WGM12); // CTC-Mode

  ICR1 = MAX; // TOP of Counter
  
  OCR1A = MIN;
  TCNT1 = 0;
  TCCR1B |= (1 << CS11);   // CPU-Takt / 8 


  spindle_stop();
}

void spindle_stop()
{
  SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
  OCR1A = MIN;
}

void spindle_run(int8_t direction, uint16_t rpm) 
{
  if (direction != current_direction) {
    plan_synchronize();
    if (direction) {
	  OCR1A = MIN + rpm;
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
