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

static void homing_cycle(bool x_axis, bool y_axis, bool z_axis, bool reverse_direction, uint32_t microseconds_per_pulse) {
  // First home the Z axis
  uint32_t step_delay = microseconds_per_pulse - settings.pulse_microseconds;
  uint8_t out_bits = DIRECTION_MASK;
  uint8_t limit_bits;
  uint8_t pulse_pin;
  
  if (x_axis) { pulse_pin = (1<<X_STEP_BIT); }
  if (y_axis) { pulse_pin = (1<<Y_STEP_BIT); }
  if (z_axis) { pulse_pin = (1<<Z_STEP_BIT); }
  
  
  DISABLE_STEPPER_DRIVER_INTERRUPT(); // This is needed because the interrupt will reset the stepping port and mess up the homing
  
  
  // This setts the direction of the homing "&= ~" sets the direction bit low "|= " sets it high.  Find a way to make this configurable from setup.c
  if (reverse_direction) {
	if (x_axis) { STEPPING_PORT &= ~(1<<X_DIRECTION_BIT); }
    if (y_axis) { STEPPING_PORT |= (1<<Y_DIRECTION_BIT); }
    if (z_axis) { STEPPING_PORT &= ~(1<<Z_DIRECTION_BIT); }
  } else {
	if (x_axis) { STEPPING_PORT |= (1<<X_DIRECTION_BIT); }
    if (y_axis) { STEPPING_PORT &= ~(1<<Y_DIRECTION_BIT); }
    if (z_axis) { STEPPING_PORT |= (1<<Z_DIRECTION_BIT); }
  }
  
  
  for(;;) {
    // This code tells when to stop based on pin state.  Find a way to make this configurable
    limit_bits = LIMIT_PIN;
    if (reverse_direction) {
	  if (x_axis && !(LIMIT_PIN & (1<<X_LIMIT_BIT))) {
        x_axis = false;
       // out_bits ^= (1<<X_STEP_BIT);      
      }
	  if (y_axis && !(LIMIT_PIN & (1<<Y_LIMIT_BIT))) {
        y_axis = false;
       // out_bits ^= (1<<Y_STEP_BIT);
      }    
      if (z_axis && !(LIMIT_PIN & (1<<Z_LIMIT_BIT))) {
        z_axis = false;
       // out_bits ^= (1<<Z_STEP_BIT);
      } 
    } else {
      if (x_axis && (LIMIT_PIN & (1<<X_LIMIT_BIT))) {
        x_axis = false;
        //out_bits ^= (1<<X_STEP_BIT);      
      }    
      if (y_axis && (LIMIT_PIN & (1<<Y_LIMIT_BIT))) {
        y_axis = false;
       // out_bits ^= (1<<Y_STEP_BIT);
      }    
      if (z_axis && (LIMIT_PIN & (1<<Z_LIMIT_BIT))) {
        z_axis = false;
       // out_bits ^= (1<<Z_STEP_BIT);
      }
	}
	
    // Check if we are done enable the stepper interrupt again and exit the loop
    if(!(x_axis || y_axis || z_axis)) { 
	  ENABLE_STEPPER_DRIVER_INTERRUPT(); 
	  return; 
	}
	
	// NOTE: set G28 to read xyz and make it step each axis individually in the order it recieves the axis commands
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

void limits_go_home(x_home, y_home, z_home) {
  st_synchronize();
  homing_cycle(x_home, y_home, z_home, false, 100);
  _delay_us(1000);
  homing_cycle(x_home, y_home, z_home, true, 10000);
}
