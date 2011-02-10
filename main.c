/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#include <avr/io.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "stepper_plan.h"
#include "stepper.h"
#include "spindle_control.h"
#include "motion_control.h"
#include "gcode.h"
#include "serial_protocol.h"

#include "settings.h"
#include "wiring_serial.h"

int main(void)
{
  sp_init(); // initialize the serial protocol
  settings_init();
  plan_init(); // initialize the stepper plan subsystem
  st_init(); // initialize the stepper subsystem
  spindle_init(); // initialize spindle controller
  gc_init(); // initialize gcode-parser
  
  for(;;){
    sleep_mode(); // Wait for it ...
    sp_process(); // ... process the serial protocol
  }
  return 0;   /* never reached */
}
