/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
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

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
   and Philipp Tiefenbacher. */

#include "stepper.h"
#include "config.h"
#include "settings.h"
#include <math.h>
#include <stdlib.h>
#include <util/delay.h>
#include "nuts_bolts.h"
#include <avr/interrupt.h>
#include "planner.h"
#include "limits.h"

// Some useful constants
#define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits
#define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits
#define STEPPING_MASK (STEP_MASK | DIRECTION_MASK) // All stepping-related bits (step/direction)

#define TICKS_PER_MICROSECOND (F_CPU/1000000)
#define CYCLES_PER_ACCELERATION_TICK ((TICKS_PER_MICROSECOND*1000000)/ACCELERATION_TICKS_PER_SECOND)

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1A) 
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1A) 

#define MINIMUM_STEPS_PER_MINUTE 1200 // The stepper subsystem will never run slower than this, exept when sleeping

static block_t *current_block;  // A pointer to the block currently being traced

bool GCMD_Trigger;

// Variables used by The Stepper Driver Interrupt
static uint8_t out_bits;        // The next stepping-bits to be output
static int32_t counter_x,       // Counter variables for the bresenham line tracer
               counter_y, 
               counter_z;       
static uint32_t step_events_completed; // The number of step events executed in the current block
static volatile int busy; // true when SIG_OUTPUT_COMPARE1A is being serviced. Used to avoid retriggering that handler.

// Variables used by the trapezoid generation
static uint32_t cycles_per_step_event;        // The number of machine cycles between each step event
static uint32_t trapezoid_tick_cycle_counter; // The cycles since last trapezoid_tick. Used to generate ticks at a steady
                                              // pace without allocating a separate timer
static uint32_t trapezoid_adjusted_rate;      // The current rate of step_events according to the trapezoid generator

//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
// 
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates by block->rate_delta
//  during the first block->accelerate_until step_events_completed, then keeps going at constant speed until 
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is always +/- block->rate_delta and is applied at a constant rate by trapezoid_generator_tick()
//  that is called ACCELERATION_TICKS_PER_SECOND times per second.

static void set_step_events_per_minute(uint32_t steps_per_minute);


// Initializes the trapezoid generator from the current block. Called whenever a new 
// block begins.
static void trapezoid_generator_reset() {
  trapezoid_adjusted_rate = current_block->initial_rate;  
  trapezoid_tick_cycle_counter = 0; // Always start a new trapezoid with a full acceleration tick
  set_step_events_per_minute(trapezoid_adjusted_rate);
}

// This is called ACCELERATION_TICKS_PER_SECOND times per second by the step_event
// interrupt. It can be assumed that the trapezoid-generator-parameters and the
// current_block stays untouched by outside handlers for the duration of this function call.
static void trapezoid_generator_tick() {     
  if (current_block) {
    if (step_events_completed < current_block->accelerate_until) {
      trapezoid_adjusted_rate += current_block->rate_delta;
      set_step_events_per_minute(trapezoid_adjusted_rate);
    } else if (step_events_completed > current_block->decelerate_after) {
      // NOTE: We will only reduce speed if the result will be > 0. This catches small
      // rounding errors that might leave steps hanging after the last trapezoid tick.
      if (trapezoid_adjusted_rate > current_block->rate_delta) {
        trapezoid_adjusted_rate -= current_block->rate_delta;
      }
      if (trapezoid_adjusted_rate < current_block->final_rate) {
        trapezoid_adjusted_rate = current_block->final_rate;
      }        
      set_step_events_per_minute(trapezoid_adjusted_rate);
    } else {
      // Make sure we cruise at exactly nominal rate
      if (trapezoid_adjusted_rate != current_block->nominal_rate) {
        trapezoid_adjusted_rate = current_block->nominal_rate;
        set_step_events_per_minute(trapezoid_adjusted_rate);
      }
    }
  }
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse of Grbl. It is  executed at the rate set with
// config_step_timer. It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
// It is supported by The Stepper Port Reset Interrupt which it uses to reset the stepper port after each pulse.
SIGNAL(TIMER1_COMPA_vect)
{        
  // TODO: Check if the busy-flag can be eliminated by just disabeling this interrupt while we are in it
  
  if(busy){ return; } // The busy-flag is used to avoid reentering this interrupt
  // Set the direction pins a cuple of nanoseconds before we step the steppers
  STEPPING_PORT = (STEPPING_PORT & ~DIRECTION_MASK) | (out_bits & DIRECTION_MASK);
  // Then pulse the stepping pins
  STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | out_bits;
  // Reset step pulse reset timer so that The Stepper Port Reset Interrupt can reset the signal after
  // exactly settings.pulse_microseconds microseconds.
  TCNT2 = -(((settings.pulse_microseconds-2)*TICKS_PER_MICROSECOND)/8);

  busy = true;
  sei(); // Re enable interrupts (normally disabled while inside an interrupt handler)
         // ((We re-enable interrupts in order for SIG_OVERFLOW2 to be able to be triggered 
         // at exactly the right time even if we occasionally spend a lot of time inside this handler.))
    
  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
    // Anything in the buffer?
    current_block = plan_get_current_block();
    if (current_block != NULL) {
      trapezoid_generator_reset();
      counter_x = -(current_block->step_event_count >> 1);
      counter_y = counter_x;
      counter_z = counter_x;
      step_events_completed = 0;
     GCMD_Trigger = true;
    } else {
		// set enable pin      
		if ((settings.enable_set == 2) || (settings.enable_set == 4)) {
		  if ((GCMD_Trigger)) {
		    st_Disable();
			GCMD_Trigger = false;
		  }
		}
	}    
  } 

  if (current_block != NULL) {
    out_bits = current_block->direction_bits;
    counter_x += current_block->steps_x;
    if (counter_x > 0) {
      out_bits |= (1<<X_STEP_BIT);
      counter_x -= current_block->step_event_count;
    }
    counter_y += current_block->steps_y;
    if (counter_y > 0) {
      out_bits |= (1<<Y_STEP_BIT);
      counter_y -= current_block->step_event_count;
    }
    counter_z += current_block->steps_z;
    if (counter_z > 0) {
      out_bits |= (1<<Z_STEP_BIT);
      counter_z -= current_block->step_event_count;
    }
    // If current block is finished, reset pointer 
    step_events_completed += 1;
    if (step_events_completed >= current_block->step_event_count) {
      current_block = NULL;
      plan_discard_current_block();
    }
  } else {
    out_bits = 0;
  }          
  out_bits ^= settings.invert_mask;
  
  // In average this generates a trapezoid_generator_tick every CYCLES_PER_ACCELERATION_TICK by keeping track
  // of the number of elapsed cycles. The code assumes that step_events occur significantly more often than
  // trapezoid_generator_ticks as they well should. 
  trapezoid_tick_cycle_counter += cycles_per_step_event;
  if(trapezoid_tick_cycle_counter > CYCLES_PER_ACCELERATION_TICK) {
    trapezoid_tick_cycle_counter -= CYCLES_PER_ACCELERATION_TICK;
    trapezoid_generator_tick();
  }
  
  busy=false;
}

// This interrupt is set up by SIG_OUTPUT_COMPARE1A when it sets the motor port bits. It resets
// the motor port after a short period (settings.pulse_microseconds) completing one step cycle.
SIGNAL(TIMER2_OVF_vect)
{
  // reset stepping pins (leave the direction pins)
  STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | (settings.invert_mask & STEP_MASK); 
}

// Initialize and start the stepper motor subsystem
void st_init()
{
	// Configure directions of interface pins
  STEPPING_DDR   |= STEPPING_MASK;
  STEPPING_PORT = (STEPPING_PORT & ~STEPPING_MASK) | settings.invert_mask;
  //STEPPERS_ENABLE_DDR |= 1<<STEPPERS_ENABLE_BIT; DVE -commented out because it is not necessary and causes motor shake at startup
  
	// waveform generation = 0100 = CTC
	TCCR1B &= ~(1<<WGM13);
	TCCR1B |=  (1<<WGM12);
	TCCR1A &= ~(1<<WGM11); 
	TCCR1A &= ~(1<<WGM10);

	// output mode = 00 (disconnected)
	TCCR1A &= ~(3<<COM1A0); 
	TCCR1A &= ~(3<<COM1B0); 
	
	// Configure Timer 2
  TCCR2A = 0;         // Normal operation
  TCCR2B = (1<<CS21); // Full speed, 1/8 prescaler
  TIMSK2 |= (1<<TOIE2);      
  
  set_step_events_per_minute(6000);
  trapezoid_tick_cycle_counter = 0;
  
   if ((settings.enable_set == 1)||(settings.enable_set == 3)) {
	st_Enable();
  } else {
    st_Disable();
  }
  sei();
}

// Block until all buffered steps are executed
void st_synchronize()
{
  while(plan_get_current_block()) { sleep_mode(); }    
}

// Configures the prescaler and ceiling of timer 1 to produce the given rate as accurately as possible.
// Returns the actual number of cycles per interrupt
static uint32_t config_step_timer(uint32_t cycles)
{
  uint16_t ceiling;
  uint16_t prescaler;
  uint32_t actual_cycles;
	if (cycles <= 0xffffL) {
		ceiling = cycles;
    prescaler = 0; // prescaler: 0
    actual_cycles = ceiling;
	} else if (cycles <= 0x7ffffL) {
    ceiling = cycles >> 3;
    prescaler = 1; // prescaler: 8
    actual_cycles = ceiling * 8L;
	} else if (cycles <= 0x3fffffL) {
		ceiling =  cycles >> 6;
    prescaler = 2; // prescaler: 64
    actual_cycles = ceiling * 64L;
	} else if (cycles <= 0xffffffL) {
		ceiling =  (cycles >> 8);
    prescaler = 3; // prescaler: 256
    actual_cycles = ceiling * 256L;
	} else if (cycles <= 0x3ffffffL) {
		ceiling = (cycles >> 10);
    prescaler = 4; // prescaler: 1024
    actual_cycles = ceiling * 1024L;    
	} else {
	  // Okay, that was slower than we actually go. Just set the slowest speed
		ceiling = 0xffff;
    prescaler = 4;
    actual_cycles = 0xffff * 1024;
	}
	// Set prescaler
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | ((prescaler+1)<<CS10);
  // Set ceiling
  OCR1A = ceiling;
  return(actual_cycles);
}

static void set_step_events_per_minute(uint32_t steps_per_minute) {
  if (steps_per_minute < MINIMUM_STEPS_PER_MINUTE) { steps_per_minute = MINIMUM_STEPS_PER_MINUTE; }
  cycles_per_step_event = config_step_timer((TICKS_PER_MICROSECOND*1000000*60)/steps_per_minute);
}

void st_go_home()
{
  limits_go_home();  
  plan_set_current_position(0,0,0);
}
// Define a function which properly configures the enable pin to turn on the stepper motors through the enable pin
void st_Enable() {
  if ((settings.enable_set == 1)||(settings.enable_set == 2)) {
	  STEPPERS_ENABLE_PORT |= (1<<STEPPERS_ENABLE_BIT);
	  ENABLE_STEPPER_DRIVER_INTERRUPT();  
  } else {
	  STEPPERS_ENABLE_PORT &= ~(1<<STEPPERS_ENABLE_BIT);
	  ENABLE_STEPPER_DRIVER_INTERRUPT();
  }
}
// Define a function which properly configures the enable pin to turn off the stepper motors through the enable pin
void st_Disable() {
  if ((settings.enable_set == 1)||(settings.enable_set == 2)) {
	  STEPPERS_ENABLE_PORT &= ~(1<<STEPPERS_ENABLE_BIT);
	  DISABLE_STEPPER_DRIVER_INTERRUPT();  
	} else {
	  STEPPERS_ENABLE_PORT |= (1<<STEPPERS_ENABLE_BIT);
	  DISABLE_STEPPER_DRIVER_INTERRUPT();
  }
}
