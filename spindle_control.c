/*
  spindle_control.c - spindle control methods
  Part of Grbl v0.9

  Copyright (c) 2012-2014 Sungeun K. Jeon

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
/* 
  This file is based on work from Grbl v0.8, distributed under the 
  terms of the MIT-license. See COPYING for more details.  
    Copyright (c) 2009-2011 Simen Svale Skogsrud
    Copyright (c) 2012 Sungeun K. Jeon
*/ 

#include "spindle_control.h"
#include "system.h"
#include "protocol.h"
#include "gcode.h"
#include "settings.h"


void spindle_init()
{    
  // On the Uno, spindle enable and PWM are shared. Other CPUs have seperate enable pin.
  #ifdef VARIABLE_SPINDLE
    SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); // Configure as PWM output pin.
    #ifndef CPU_MAP_ATMEGA328P 
      SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
    #endif     
  #else
    SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
  #endif
  SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.
  spindle_stop();
}


void spindle_stop()
{
  // On the Uno, spindle enable and PWM are shared. Other CPUs have seperate enable pin.
  #ifdef VARIABLE_SPINDLE
    TCCRA_REGISTER &= ~(1<<COMB_BIT); // Disable PWM. Output voltage is zero.
    #ifndef CPU_MAP_ATMEGA328P 
      SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low.
    #endif
  #else
    SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low.
  #endif  
}

#ifdef VARIABLE_SPINDLE
  void spindle_start()
  {
    // TODO: Install the optional capability for frequency-based output for servos.
    TCCRA_REGISTER = (1<<COMB_BIT) | (1<<WAVE1_REGISTER) | (1<<WAVE0_REGISTER);
    TCCRB_REGISTER = (TCCRB_REGISTER & 0b11111000) | 0x02; // set to 1/8 Prescaler
  }

  void spindle_rpm_update(uint8_t pwm)
  {
    // TODO: Install the optional capability for frequency-based output for servos.
    OCR_REGISTER = pwm;
  }

  uint8_t calculate_pwm_from_rpm(float rpm)
  {
     // TODO: Install the optional capability for frequency-based output for servos.
     #define SPINDLE_RPM_RANGE (SPINDLE_MAX_RPM-SPINDLE_MIN_RPM)
     rpm -= SPINDLE_MIN_RPM;
     if ( rpm > SPINDLE_RPM_RANGE ) { rpm = SPINDLE_RPM_RANGE; } // Prevent uint8 overflow
     return (uint8_t) floor( rpm*(255.0/SPINDLE_RPM_RANGE) + 0.5);
  }
#endif

void spindle_run(uint8_t direction, float rpm, uint8_t motion) 
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  // Empty planner buffer to ensure spindle is set when programmed.
  // Only do this without real time spindle control or if there was
  // a block without motion.  
  #ifdef LASER_SPINDLE
    if (bit_isfalse(settings.flags,BITFLAG_LASER) || motion == BLOCK_HAS_NO_MOTION)
  #endif
  protocol_auto_cycle_start();  //temp fix for M3 lockup
  #ifdef LASER_SPINDLE
    if (bit_isfalse(settings.flags,BITFLAG_LASER) || motion == BLOCK_HAS_NO_MOTION)
  #endif
  protocol_buffer_synchronize();   
  // Halt or set spindle direction and rpm. 
  if (direction == SPINDLE_DISABLE) {
    spindle_stop();
  } else {
    if (direction == SPINDLE_ENABLE_CW) {
      SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
    } else {
      SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
    }
    #ifdef VARIABLE_SPINDLE
       spindle_start();
       spindle_rpm_update(calculate_pwm_from_rpm(rpm));
      #ifndef CPU_MAP_ATMEGA328P // On the Uno, spindle enable and PWM are shared.
        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
      #endif
    #else   
      SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
    #endif
  }
}
