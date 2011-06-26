/*
  limits.h - code pertaining to limit-switches and performing the homing cycle
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
  
#include <util/delay.h>
#include <avr/io.h>
#include "stepper.h"
#include "settings.h"
#include "nuts_bolts.h"
#include "config.h"


void limits_init() {
  LIMIT_DDR &= ~(LIMIT_MASK);
}

static void homing_cycle(uint8_t Axis_Select, bool reverse_direction, uint32_t microseconds_per_pulse) {
  // First home the Z axis
  uint32_t step_delay = microseconds_per_pulse - settings.pulse_microseconds;
  uint8_t out_bits = DIRECTION_MASK;
  uint8_t pulse_pin;
  bool stop_loop = false;
  bool limit_pin_cont[3];
  bool limit_dir_cont[3];
  uint8_t limit_bit_cont[3];
  uint8_t direction_bit_cont[3];
  uint8_t step_bit_cont[3];
  
  limit_dir_cont[X_AXIS] = true;
  limit_dir_cont[Y_AXIS] = false;
  limit_dir_cont[Z_AXIS] = true;
  
  limit_bit_cont[X_AXIS] = X_LIMIT_BIT;
  limit_bit_cont[Y_AXIS] = Y_LIMIT_BIT;
  limit_bit_cont[Z_AXIS] = Z_LIMIT_BIT;
  
  direction_bit_cont[X_AXIS] = X_DIRECTION_BIT;
  direction_bit_cont[Y_AXIS] = Y_DIRECTION_BIT;
  direction_bit_cont[Z_AXIS] = Z_DIRECTION_BIT;
  
  step_bit_cont[X_AXIS] = X_STEP_BIT;
  step_bit_cont[Y_AXIS] = Y_STEP_BIT;
  step_bit_cont[Z_AXIS] = Z_STEP_BIT;
  
  // set the step bit
  pulse_pin = (1<<step_bit_cont[Axis_Select]);
  
  st_Enable(); // make sure steppers are turned on
  DISABLE_STEPPER_DRIVER_INTERRUPT(); // This is needed because the interrupt will reset the stepping port and mess up the homing
  
  
  // This setts the direction of the homing
  if (reverse_direction) {
	if (limit_dir_cont[Axis_Select]) {
	  STEPPING_PORT &= ~(1<<direction_bit_cont[Axis_Select]);
	} else {
	  STEPPING_PORT |= (1<<direction_bit_cont[Axis_Select]);
	}
  } else {
    if (limit_dir_cont[Axis_Select]) {
	  STEPPING_PORT |= (1<<direction_bit_cont[Axis_Select]);
	} else {
	  STEPPING_PORT &= ~(1<<direction_bit_cont[Axis_Select]);
	}
  }
  
  
  for(;;) {
	// detect limit pin
	limit_pin_cont[Axis_Select] = (LIMIT_PIN & (1<<limit_bit_cont[Axis_Select]));
	
	// reverse pin state when backing off limit switch
    if (reverse_direction) {
	  limit_pin_cont[Axis_Select] = !limit_pin_cont[Axis_Select];
    }

	// detect when to exit loop
	if (limit_pin_cont[Axis_Select]) {
	  stop_loop = true;
	}
	
    // Check if we are done enable the stepper interrupt again and exit the loop
    if(stop_loop) { 
	  ENABLE_STEPPER_DRIVER_INTERRUPT(); 
	  if ((settings.enable_set == 2) || (settings.enable_set == 4)) {
	    st_Disable();
	  }
	  return; 
	}
	
	// pulse the steppers to make them move
    STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | (out_bits & pulse_pin); // pulse the stepper motors
    _delay_us(settings.pulse_microseconds);  // wait required time
    STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | (~out_bits & pulse_pin); // reset pins while leaving direction pins alone
    _delay_us(step_delay);  // wait a given period to move again
  }
  return;
}

// static void approach_limit_switch(bool x, bool y, bool z) {
//  homing_cycle(x, y, z, false, 100); // NOTE: make the pulse period configurable and calculate based on velocity
//}

//static void leave_limit_switch(bool x, bool y, bool z) {
//  homing_cycle(x, y, z, true, 10000);  // NOTE: make the pulse period configurable and calculate based on velocity
//}

void limits_go_home(Axis_Select) {
  st_synchronize();
  homing_cycle(Axis_Select, false, 100);
  _delay_us(1000);
  homing_cycle(Axis_Select, true, 10000);
}
