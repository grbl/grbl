/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2015 Sungeun K. Jeon
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

#include "grbl.h"


void spindle_init()
{    
  // Configure variable spindle PWM and enable pin, if required.
  SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); // Configure as PWM output pin.
  SPINDLE_TCCRA_REGISTER = SPINDLE_TCCRA_INIT_MASK; // Configure PWM output compare timer
  SPINDLE_TCCRB_REGISTER = SPINDLE_TCCRB_INIT_MASK;
  SPINDLE_OCRA_REGISTER = SPINDLE_OCRA_TOP_VALUE; // Set the top value for 16-bit fast PWM mode
  SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
  SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.

  spindle_stop();
}


uint8_t spindle_is_enabled()
{
  #ifdef INVERT_SPINDLE_ENABLE_PIN
    if ((SPINDLE_ENABLE_PORT) & (1<<SPINDLE_ENABLE_BIT)) { return(true); }
    else { return(false); }
  #else
    if ((SPINDLE_ENABLE_PORT) & (1<<SPINDLE_ENABLE_BIT)) { return(false); }
    else { return(true); }
  #endif
}


void spindle_stop()
{
  SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); // Disable PWM. Output voltage is zero.
  #ifdef INVERT_SPINDLE_ENABLE_PIN
    SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  // Set pin to high
  #else
    SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
  #endif
}


void spindle_set_state(uint8_t state, float rpm)
{
  if (sys.abort) { return; } // Block during abort.
  
  // Halt or set spindle direction and rpm. 
  if (state == SPINDLE_DISABLE) {

    spindle_stop();

  } else {

    if (state == SPINDLE_ENABLE_CW) {
      SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
    } else {
      SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
    }

    uint16_t current_pwm; // 2560 PWM register is 16-bit.

    // Calculate PWM register value based on rpm max/min settings and programmed rpm.
    if (rpm <= 0.0) { spindle_stop(); } // RPM should never be negative, but check anyway.
    else {
      if (settings.rpm_max <= settings.rpm_min) {
        // No PWM range possible. Set simple on/off spindle control pin state.
        current_pwm = SPINDLE_PWM_MAX_VALUE;
      } else {
        if (rpm > settings.rpm_max) { rpm = settings.rpm_max; }
        if (rpm < settings.rpm_min) { rpm = settings.rpm_min; }
        #ifdef SPINDLE_MINIMUM_PWM
          float pwm_gradient = (SPINDLE_PWM_MAX_VALUE-SPINDLE_MINIMUM_PWM)/(settings.rpm_max-settings.rpm_min);
          current_pwm = floor( (rpm-settings.rpm_min)*pwm_gradient + (SPINDLE_MINIMUM_PWM+0.5));
        #else
          float pwm_gradient = (SPINDLE_PWM_MAX_VALUE)/(settings.rpm_max-settings.rpm_min);
          current_pwm = floor( (rpm-settings.rpm_min)*pwm_gradient + 0.5);
        #endif
      }
   
      SPINDLE_OCR_REGISTER = current_pwm; // Set PWM output level.
      SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // Ensure PWM output is enabled.
 
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
      #else
        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
      #endif
    }
  
  }
}


void spindle_run(uint8_t state, float rpm)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.  
  spindle_set_state(state, rpm);
}
