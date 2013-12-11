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

#include "settings.h"
#include "spindle_control.h"
#include "planner.h"

static uint8_t current_direction;
static uint16_t current_rpm;

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
#ifdef SPINDLE_IS_STEPPER
                
	float frequency = ((float) SPINDLE_STEPS_PER_REV * (float) rpm )/ 60.0;

	struct {
		uint8_t prescaler;
		uint16_t divisor;
		uint8_t bits;
	}clockConfig[] = {
			{ 1, 0, 0x01},
			{ 2, 8, 0x02},
			{ 3, 64, 0x03},
			{ 4, 256, 0x04},
			{ 5, 1024, 0x05}
	};

	float ticks;
	float ticklow = 32257.0;
	float error;
	uint8_t clkId = 4;
	uint8_t bestClockid = 4;
	float bestError = 1.0;

	do {
		ticks = (float) F_CPU / frequency / (float) clockConfig[clkId].divisor;
		error = ((ticks - round(ticks))>0?(ticks - round(ticks)):-(ticks - round(ticks)));
		if (error < bestError)
			{
				bestClockid = clkId;
				bestError = error;
			}
	}while (clkId-- > 0);
		ticks = (float) F_CPU / frequency / (float) clockConfig[bestClockid].divisor;

			if (frequency > 10000) ticklow = ticks * 0.6;
			if (frequency > 5000 && frequency <= 10000) ticklow = ticks * 0.7;
			if (frequency > 2000 && frequency <= 5000) ticklow = ticks * 0.8;
			if (frequency <= 2000) ticklow = ticks * 0.9;

 	TCCRA_REGISTER = (1<<COMB_BIT) | (1<<WAVE1_REGISTER) | (1<<WAVE0_REGISTER);
	TCCRB_REGISTER = (1<<WAVE2_REGISTER) | (1<<WAVE3_REGISTER);
	// Set prescaler
	TCCRB_REGISTER = (TCCRB_REGISTER & 0b11111000) | clockConfig[bestClockid].bits;
	OCRA_REGISTER = floor(ticks + 0.5);
	OCRB_REGISTER = floor(ticklow + 0.5);
#endif
#ifdef SPINDLE_IS_PWM

	TCCRA_REGISTER = (1<<COMB_BIT) | (1<<WAVE1_REGISTER) | (1<<WAVE0_REGISTER);
	TCCRB_REGISTER = (1<<WAVE2_REGISTER) | (1<<WAVE3_REGISTER);
	// Set prescaler it is currently set at 1/8 ( 2MHz )
	// I have set it up this way so it can be easily adjusted if a certain frequency is required
	// eg. if you require a 10kHz frequency change prescaler to 0x05 and OCRA_REGISTER to 41942
	// you will also need to change the last value of OCRB_REGISTER to match OCRA_REGISTER ie. 41942
	TCCRB_REGISTER = (TCCRB_REGISTER & 0b11111000) | 0x02; // set to 1/8 Prescaler
	OCRA_REGISTER = 65535;
	OCRB_REGISTER = ((float) rpm / (float) SPINDLE_MAX_RPM ) * 65535.0;
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