/*
 probe.c - code pertaining to perform probing in Grbl

 Copyright (c) 2013 Henrik Olsson

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

#include "probe.h"
#include "stepper.h"
#include "planner.h"
#include "pin_map.h"

#ifdef PROBE_38
void probe_init()
{
  PROBE_DDR &= ~(PROBE_MASK); // Set as input pins
  PROBE_PORT |= PROBE_MASK; // Enable internal pull-up resistors. Normal high operation.
  PROBE_PCMSK |= PROBE_MASK; // Enable specific pins of the Pin Change Interrupt
  PCICR |= (1 << PROBE_INT);   // Enable Pin Change Interrupt
}

void probe_ISR()
{
  plan_block_t *block = plan_get_current_block();
  if (block && block->probing && bit_isfalse(block->probing, PROBING_TOUCH))
  {
    if ((bit_isfalse(PROBE_PIN,bit(PIN_PROBE)) && bit_istrue(block->probing ,PROBING_TO))
    || (bit_istrue(PROBE_PIN,bit(PIN_PROBE)) && bit_isfalse(block->probing,PROBING_TO)))
    {
      bit_true(block->probing, PROBING_TOUCH);
      bit_true(block->probing, PROBING_HANDELD);
      st_feed_hold();
      memcpy(sys.probe_position,sys.position,sizeof(sys.position));
      bit_true(sys.execute, EXEC_PROBE_REPORT);
    }
  }
}
#endif
