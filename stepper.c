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
   and Philipp Tiefenbacher. The ring buffer implementation gleaned from the wiring_serial library 
   by David A. Mellis */

#include "stepper.h"
#include "config.h"
#include <math.h>
#include <stdlib.h>
#include <util/delay.h>
#include "nuts_bolts.h"
#include <avr/interrupt.h>
#include "stepper_plan.h"
#include "wiring_serial.h"

void set_step_events_per_minute(uint32_t steps_per_minute);

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1A)

#define CYCLES_PER_ACCELERATION_TICK ((TICKS_PER_MICROSECOND*1000000)/ACCELERATION_TICKS_PER_SECOND)

struct Block *current_block;    // A convenience pointer to the block currently being traced

// Variables used by The Stepper Driver Interrupt
uint8_t out_bits;               // The next stepping-bits to be output
int32_t counter_x, 
        counter_y, 
        counter_z;              // counter variables for the bresenham line tracer
uint32_t iterations;            // The number of iterations left to complete the current_block
volatile int busy;              // TRUE when SIG_OUTPUT_COMPARE1A is being serviced. Used to avoid retriggering that handler.
uint32_t cycles_per_step_event;
uint32_t trapezoid_tick_cycle_counter;

// Values and variables used by the speed trapeziod generator  
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
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates for 
//  block->accelerate_ticks by block->rate_delta each tick, then stays up for block->plateau_ticks and 
//  decelerates for the rest of the block until the trapezoid generator is reset for the next block. 
//  The slope of acceleration is always +/- block->rate_delta. Any stage may be skipped by setting the 
//  duration to 0 ticks. 
     
#define TRAPEZOID_STAGE_ACCELERATING 0
#define TRAPEZOID_STAGE_PLATEAU 1
#define TRAPEZOID_STAGE_DECELERATING 2
uint8_t trapezoid_stage;
uint16_t trapezoid_stage_ticks;
uint32_t trapezoid_rate;
int16_t trapezoid_delta;


// Initializes the trapezoid generator from the current block. Called whenever a new 
// block begins.
inline void reset_trapezoid_generator() {      
  trapezoid_stage = TRAPEZOID_STAGE_ACCELERATING;
  trapezoid_stage_ticks = current_block->accelerate_ticks;
  trapezoid_delta = current_block->rate_delta;
  trapezoid_rate = current_block->initial_rate;  
  set_step_events_per_minute(trapezoid_rate);
}

// This is called ACCELERATION_TICKS_PER_SECOND times per second by the step_event
// interrupt. It can be assumed that the trapezoid-generator-parameters and the
// current_block stays untouched by outside handlers for the duration of this function call.
inline void trapezoid_generator_tick() {     
  if (trapezoid_stage_ticks) {
    trapezoid_stage_ticks--;
    if (trapezoid_delta) {
      trapezoid_rate += trapezoid_delta;
      set_step_events_per_minute(trapezoid_rate);
    }
  } else {
    // Is there a block currently in execution?
    if(!current_block) {return;}    
    // Trapezoid stage complete, move on
    if(trapezoid_stage == TRAPEZOID_STAGE_ACCELERATING) {
      // Progress to plateau stage
      trapezoid_delta = 0;
      trapezoid_stage_ticks = current_block->plateau_ticks;
      trapezoid_stage = TRAPEZOID_STAGE_PLATEAU;
    } else if (trapezoid_stage == TRAPEZOID_STAGE_PLATEAU) {
      // Progress to deceleration stage
      trapezoid_delta = -current_block->rate_delta;
      trapezoid_stage_ticks = 0xffff; // "forever" until the block is complete
      trapezoid_stage = TRAPEZOID_STAGE_DECELERATING;
    }
  }
}

// Add a new linear movement to the buffer. steps_x, _y and _z is the signed, relative motion in 
// steps. Microseconds specify how many microseconds the move should take to perform. To aid acceleration
// calculation the caller must also provide the physical length of the line in millimeters.
void st_buffer_line(int32_t steps_x, int32_t steps_y, int32_t steps_z, uint32_t microseconds, double millimeters) {
  plan_buffer_line(steps_x, steps_y, steps_z, microseconds, millimeters);
  // Ensure that block processing is running by enabling The Stepper Driver Interrupt
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse of Grbl. It is  executed at the rate set with
// config_step_timer. It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
// It is supported by The Stepper Port Reset Interrupt which it uses to reset the stepper port after each pulse.
#ifdef TIMER1_COMPA_vect
SIGNAL(TIMER1_COMPA_vect)
#else
SIGNAL(SIG_OUTPUT_COMPARE1A)
#endif
{
  if(busy){ return; } // The busy-flag is used to avoid reentering this interrupt
  
  // Set the direction pins a cuple of nanoseconds before we step the steppers
  STEPPING_PORT = (STEPPING_PORT & ~DIRECTION_MASK) | (out_bits & DIRECTION_MASK);
  // Then pulse the stepping pins
  STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | out_bits;
  // Reset step pulse reset timer so that The Stepper Port Reset Interrupt can reset the signal after
  // exactly settings.pulse_microseconds microseconds.
  TCNT2 = -(((settings.pulse_microseconds-2)*TICKS_PER_MICROSECOND)/8);

  busy = TRUE;
  sei(); // Re enable interrupts (normally disabled while inside an interrupt handler)
         // ((We re-enable interrupts in order for SIG_OVERFLOW2 to be able to be triggered 
         // at exactly the right time even if we occasionally spend a lot of time inside this handler.))
    
  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
    // Anything in the buffer?
    if (block_buffer_head != block_buffer_tail) {
      // Retrieve a new line and get ready to step it
      current_block = &block_buffer[block_buffer_tail]; 
      reset_trapezoid_generator();
      counter_x = -(current_block->step_event_count >> 1);
      counter_y = counter_x;
      counter_z = counter_x;
      iterations = current_block->step_event_count;
    } else {
      DISABLE_STEPPER_DRIVER_INTERRUPT();
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
    iterations -= 1;
    if (iterations <= 0) {
      current_block = NULL;
      // move the block buffer tail to the next instruction
      block_buffer_tail = (block_buffer_tail + 1) % BLOCK_BUFFER_SIZE;      
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
  
  busy=FALSE;
}

// This interrupt is set up by SIG_OUTPUT_COMPARE1A when it sets the motor port bits. It resets
// the motor port after a short period (settings.pulse_microseconds) completing one step cycle.
#ifdef TIMER2_OVF_vect
SIGNAL(TIMER2_OVF_vect)
#else
SIGNAL(SIG_OVERFLOW2)
#endif
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
  LIMIT_DDR &= ~(LIMIT_MASK);
  STEPPERS_ENABLE_DDR |= 1<<STEPPERS_ENABLE_BIT;
  
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

  DISABLE_STEPPER_DRIVER_INTERRUPT();
  
  // set enable pin     
  STEPPERS_ENABLE_PORT |= 1<<STEPPERS_ENABLE_BIT;
  
  sei();
}

// Block until all buffered steps are executed
void st_synchronize()
{
  while(block_buffer_tail != block_buffer_head) { sleep_mode(); }    
}

// Cancel all buffered steps
void st_flush()
{
  cli();
  block_buffer_tail = block_buffer_head;
  current_block = NULL;
  sei();
}

// Configures the prescaler and ceiling of timer 1 to produce the given rate as accurately as possible.
// Returns the actual number of cycles per interrupt
uint32_t config_step_timer(uint32_t cycles)
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
    actual_cycles = ceiling * 8;
	} else if (cycles <= 0x3fffffL) {
		ceiling =  cycles >> 6;
    prescaler = 2; // prescaler: 64
    actual_cycles = ceiling * 64;
	} else if (cycles <= 0xffffffL) {
		ceiling =  (cycles >> 8);
    prescaler = 3; // prescaler: 256
    actual_cycles = ceiling * 256;
	} else if (cycles <= 0x3ffffffL) {
		ceiling = (cycles >> 10);
    prescaler = 4; // prescaler: 1024
    actual_cycles = ceiling * 1024;    
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

void set_step_events_per_minute(uint32_t steps_per_minute) {
  cycles_per_step_event = config_step_timer((TICKS_PER_MICROSECOND*1000000*60)/steps_per_minute);
}

void st_go_home()
{
  // Todo: Perform the homing cycle
}
