/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2014 Sungeun K. Jeon
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

#include "settings.h"
#include "spindle_control.h"
#include "planner.h"

static uint8_t current_direction;
static uint16_t current_rpm;


void spindle_init()
{
  current_direction = 0;
  SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT);
  
  // On the Uno, spindle enable and PWM are shared. Other CPUs have seperate enable pin.
  #ifdef VARIABLE_SPINDLE
    SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT);
    #ifndef CPU_MAP_ATMEGA328P 
      SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
    #endif     
  #else
    SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
  #endif
  
  spindle_stop();
}


void spindle_stop()
{
  // On the Uno, spindle enable and PWM are shared. Other CPUs have seperate enable pin.
  #ifdef VARIABLE_SPINDLE
    TCCRA_REGISTER &= ~(1<<COMB_BIT); // Disable PWM. Output voltage is zero.
    #ifndef CPU_MAP_ATMEGA328P 
      SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
    #endif
  #else
    SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
  #endif  
}


void spindle_run(int8_t direction, uint16_t rpm) 
{
  if ((direction != current_direction) || (rpm != current_rpm)) {
    plan_synchronize(); // Empty planner buffer to ensure spindle is updated as programmed.
    if (direction) {
      if (direction > 0) {
        SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
      } else {
        SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
      }

      #ifdef VARIABLE_SPINDLE
        TCCRA_REGISTER = (1<<COMB_BIT) | (1<<WAVE1_REGISTER) | (1<<WAVE0_REGISTER);
        TCCRB_REGISTER = (TCCRB_REGISTER & 0b11111000) | 0x02; // set to 1/8 Prescaler
        if (rpm > SPINDLE_MAX_RPM) { rpm = SPINDLE_MAX_RPM; } // Prevent overflow.
        uint8_t current_pwm = floor((((float) rpm / (float) SPINDLE_MAX_RPM ) * 255.0) + 0.5);
        OCR_REGISTER = current_pwm;
        
        #ifndef CPU_MAP_ATMEGA328P // On the Uno, spindle enable and PWM are shared.
          SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
        #endif
      #else   
        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
      #endif
          
	} else {
      spindle_stop();
	}
    current_direction = direction;
    current_rpm = rpm;
  }
}

#endif
