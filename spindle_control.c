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

#ifdef VARIABLE_SPINDLE
static uint8_t current_pwm; // Stores the PWM set by the S value
#endif 

void spindle_init()
{
  current_direction = 0;
  SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT);
  SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT);
  
#ifdef VARIABLE_SPINDLE
        SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT);
#endif 
  
  spindle_stop();
}

void spindle_stop()
{
#ifdef INVERT_SPINDLE
        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
#else
        SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
#endif
#ifdef VARIABLE_SPINDLE
        TCCRA_REGISTER &= ~(1<<COMB_BIT);
#endif
}

void spindle_run(int8_t direction, uint16_t rpm) {
	if ((direction != current_direction) || (rpm != current_rpm)) {
		plan_synchronize();
	if (direction) {
		if (direction > 0) {
			SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
		} else {
			SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
		}

#ifdef VARIABLE_SPINDLE
#ifdef SPINDLE_IS_PWM

	TCCRA_REGISTER = (1<<COMB_BIT) | (1<<WAVE1_REGISTER) | (1<<WAVE0_REGISTER);
	TCCRB_REGISTER = (TCCRB_REGISTER & 0b11111000) | 0x02; // set to 1/8 Prescaler
	current_pwm = floor((((float) rpm / (float) SPINDLE_MAX_RPM ) * 255.0) + 0.5);
	OCR_REGISTER = current_pwm;
	
#endif
#endif
#ifdef INVERT_SPINDLE
        SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
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

#ifdef VARIABLE_SPINDLE
uint8_t spindle_pwm()
{
	// This function can be used by the st_prep_buffer() to calculate what the spindle speed vs travel speed
	return current_pwm;
}

void spindle_pwm_update(uint8_t pwm)
{
	// This function can be used by the stepper interrupt to set the spindle speed
	OCR_REGISTER = pwm;

}
#endif