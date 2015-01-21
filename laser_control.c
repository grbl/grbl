/*
  laser_control.c - laser control methods
  Part of Horus Firmware

  Copyright (c) 2014-2105 Jesus Arroyo (Mundo Reader S.L.)

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

// Used to count 1 second with timer2
volatile uint8_t timer = 0;

// Laser status array
volatile uint8_t laser[4];

// Laser timer array
volatile uint8_t laser_timer[4];

void laser_init()
{
  // Initialize lasers
  LASER_DDR |= LASER_MASK;
  LASER_PORT &= ~LASER_MASK;

  // Initialize timer2
  cli();                              // disable interrupts
  TCNT2  = 0;                         // initialize counter value
  OCR2A = 244-1;                      // compare match register 16MHz/256/256Hz
  TCCR2A = (1 << WGM21);              // CTC mode
  TCCR2B = (1 << CS22) | (1 << CS21); // 256 prescaler
  TIMSK2 |= (1 << OCIE2A);            // enable timer compare interrupt
  sei();                              // enable interrupts
}

void laser_on(uint8_t laser_bit)
{
  LASER_PORT |= laser_bit;
}

void laser_off(uint8_t laser_bit)
{
  LASER_PORT &= ~laser_bit;
}

void laser_set(uint8_t id, uint8_t value)
{
  uint8_t bit = 0;

  switch (id) {
    case 0: bit = (1<<LASER1_BIT); break;
    case 1: bit = (1<<LASER2_BIT); break;
    case 2: bit = (1<<LASER3_BIT); break;
    case 3: bit = (1<<LASER4_BIT); break;
  }
  
  if (bit > 0) {
    if (value == LASER_ENABLE) {
      laser_on(bit);
      laser[id] = 1;
    } else {
      laser_off(bit);
      laser[id] = 0;
      laser_timer[id] = 0;
    }
  }
}

void laser_run(uint8_t id, uint8_t value)
{
  if (sys.state == STATE_CHECK_MODE) { return; }

  protocol_auto_cycle_start();
  protocol_buffer_synchronize();

  laser_set(id-1, value);
}

ISR(TIMER2_COMPA_vect)
{
  if (!++timer) { // 1 second reached!
    uint8_t i;
    for (i = 0; i < 4; i++) {
      if (laser[i]) laser_timer[i]++;
      if (laser_timer[i] == 255) // Laser i disabled after 255 seconds max
        laser_set(i, LASER_DISABLE);
    }
  }
}