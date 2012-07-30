/*
  cpump.c - charge pump methods
  Part of Grbl

  Written by Radu - Eosif Mihailescu in July 2012.
  I choose to claim no copyright over the contents of this file. This would
  normally mean I'm placing it in the public domain however, in this case, the
  project-wide license (GNU GPL) applies. NOTE: IANAL, so ask yours if in doubt.

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

#include "cpump.h"

#include <avr/io.h>

void cpump_init()
{
#ifdef CHARGE_PUMP
  OCR0A = F_CPU / (25000UL << 3); // Faster than multiplying by 8
  TCCR0A = _BV(WGM02); // CTC mode
  TCCR0B = _BV(COM0A0) | _BV(CS01); // Toggle OC2A on compare and 1/8 prescaler
  TIMSK0 = 0; // No interrupts
#endif
}
