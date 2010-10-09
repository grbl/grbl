/*
  stepper.c - stepper motor interface
  Part of Grbl

  Copyright (c) 2009 Simen Svale Skogsrud

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

#include "wiring_serial.h"

// Pick a suitable block-buffer size
#ifdef __AVR_ATmega328P__
#define BLOCK_BUFFER_SIZE 40   // Atmega 328 has one full kilobyte of extra RAM!
#else
#define BLOCK_BUFFER_SIZE 10
#endif

// This record is used to buffer the setup for each motion block
struct Block {
  uint32_t steps_x, steps_y, steps_z; // Step count along each axis
  uint32_t pos_x, pos_y, pos_z;		  // Initial position on line
//  double rate_x, rate_y, rate_z;      // Nominal steps/second for each axis
  int32_t  maximum_steps;             // The largest stepcount of any axis for this block
  uint8_t  direction_bits;            // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  uint32_t rate;                      // The nominal step rate for this block in microseconds/step
};

extern int32_t position[3];    // The current position of the tool in absolute steps

int32_t actual_position[3];    // The current position of the tool in absolute steps
						// In this file, this is actually the current position, not 
						// estimated current position...


struct Block block_buffer[BLOCK_BUFFER_SIZE]; // A buffer for step instructions
volatile int block_buffer_head = 0;
volatile int block_buffer_tail = 0;

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1A)


// Variables used by SIG_OUTPUT_COMPARE1A
uint8_t out_bits;               // The next stepping-bits to be output
struct Block *current_block;    // A pointer to the block currently being traced
volatile int32_t counter_x, 
                 counter_y, 
                 counter_z;     // counter variables for the bresenham line tracer
uint32_t iterations;            // The number of iterations left to complete the current_block
volatile int busy;              // TRUE when SIG_OUTPUT_COMPARE1A is being serviced. Used to avoid retriggering that handler.

void config_step_timer(uint32_t microseconds);

// Add a new linear movement to the buffer. steps_x, _y and _z is the signed, relative motion in 
// steps. Microseconds specify how many microseconds the move should take to perform.
void st_buffer_block(int32_t steps_x, int32_t steps_y, int32_t steps_z,
					int32_t pos_x,   int32_t pos_y,   int32_t pos_z,
					uint32_t microseconds) {
  // Calculate the buffer head after we push this byte
	int next_buffer_head = (block_buffer_head + 1) % BLOCK_BUFFER_SIZE;	
	// If the buffer is full: good! That means we are well ahead of the robot. 
	// Nap until there is room in the buffer.
  while(block_buffer_tail == next_buffer_head) { sleep_mode(); }
  // Setup block record
  struct Block *block = &block_buffer[block_buffer_head];
  block->steps_x = labs(steps_x);
  block->steps_y = labs(steps_y);
  block->steps_z = labs(steps_z);  
    
  block->pos_x = pos_x;
  block->pos_y = pos_y;
  block->pos_z = pos_z;  
  
  block->maximum_steps = max(block->steps_x, max(block->steps_y, block->steps_z));
  // Bail if this is a zero-length block
  if (block->maximum_steps == 0) { return; };
  // Rate in steps/second for each axis
//  double rate_multiplier = 1000000.0/microseconds;
//  block->rate_x = block->steps_x*rate_multiplier;
//  block->rate_y = block->steps_y*rate_multiplier;
//  block->rate_z = block->steps_z*rate_multiplier;
  block->rate = microseconds/block->maximum_steps;
  // Compute direction bits for this block
  uint8_t direction_bits = 0;
  if (steps_x < 0) { direction_bits |= (1<<X_DIRECTION_BIT); }
  if (steps_y < 0) { direction_bits |= (1<<Y_DIRECTION_BIT); }
  if (steps_z < 0) { direction_bits |= (1<<Z_DIRECTION_BIT); }
  block->direction_bits = direction_bits;
  // Move buffer head
  block_buffer_head = next_buffer_head;
  // Ensure that blocks will be processed by enabling The Stepper Driver Interrupt
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse of Trbl. It is  executed at the rate set with
// config_step_timer. It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
// It is supported by The Stepper Port Reset Interrupt which it uses to reset the stepper port after each pulse.
#ifdef TIMER1_COMPA_vect
SIGNAL(TIMER1_COMPA_vect)
#else
SIGNAL(SIG_OUTPUT_COMPARE1A)
#endif
{
  if(busy){ return; } // The busy-flag is used to avoid reentering this interrupt
  STEPPERS_ENABLE_PORT |= (1<<STEPPERS_ENABLE_BIT);
  // Set the direction pins a cuple of nanoseconds before we step the steppers
  STEPPING_PORT = (STEPPING_PORT & ~DIRECTION_MASK) | (out_bits & DIRECTION_MASK);
  // Then pulse the stepping pins
  STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | out_bits;
  // Reset step pulse reset timer so that SIG_OVERFLOW2 can reset the signal after
  // exactly settings.pulse_microseconds microseconds.
  TCNT2 = -(((settings.pulse_microseconds-2)*TICKS_PER_MICROSECOND)/8);

  busy = TRUE;
  sei(); // Re enable interrupts (normally disabled while inside an interrupt handler)
 		 // We re-enable interrupts in order for SIG_OVERFLOW2 to be able to be triggered 
 		 // at exactly the right time even if we occasionally spend a lot of time inside this handler.
        
  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
    // PORTD &= ~(1<<4); // Can't figure out what this does (bob)
    // Anything in the buffer?
    if (block_buffer_head != block_buffer_tail) {
      // PORTD ^= (1<<5); // Can't figure out what this does (bob)
      // Retrieve a new block and get ready to step it
      current_block = &block_buffer[block_buffer_tail]; 
      config_step_timer(current_block->rate);
      counter_x = -(current_block->maximum_steps >> 1);
      counter_y = counter_x;
      counter_z = counter_x;
      actual_position[X_AXIS] = current_block->pos_x;
      actual_position[Y_AXIS] = current_block->pos_y;
      actual_position[Z_AXIS] = current_block->pos_z;
      iterations = current_block->maximum_steps;
    } else {
      // Buffer empty. Disable this interrupt until there is something to handle
      DISABLE_STEPPER_DRIVER_INTERRUPT();
	  STEPPERS_ENABLE_PORT &= ~(1<<STEPPERS_ENABLE_BIT);
    }    
  } 

  if (current_block != NULL) {
    out_bits = current_block->direction_bits;
    counter_x += current_block->steps_x;
    if (counter_x > 0) {
      out_bits |= (1<<X_STEP_BIT);
      counter_x -= current_block->maximum_steps;
      if (out_bits & (1<<X_DIRECTION_BIT)){
      	actual_position[X_AXIS]-=1;
      } else {
      	actual_position[X_AXIS]+=1;
      }
    }
    counter_y += current_block->steps_y;
    if (counter_y > 0) {
      out_bits |= (1<<Y_STEP_BIT);
      counter_y -= current_block->maximum_steps;
      if (out_bits & (1<<Y_DIRECTION_BIT)){
      	actual_position[Y_AXIS]-=1;
      } else {
      	actual_position[Y_AXIS]+=1;
      }
    }
    counter_z += current_block->steps_z;
    if (counter_z > 0) {
      out_bits |= (1<<Z_STEP_BIT);
      counter_z -= current_block->maximum_steps;
      if (out_bits & (1<<Z_DIRECTION_BIT)){
      	actual_position[Z_AXIS]-=1;
      } else {
      	actual_position[Z_AXIS]+=1;
      }
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
  busy=FALSE;
  // Done. The next time this interrupt is entered the out_bits we just calculated will be pulsed onto the port
}


// "The Stepper Port Reset Interrupt" - This interrupt is set up by The Stepper Driver Interrupt when it sets the 
// motor port bits. It resets the motor port after a short period (settings.pulse_microseconds) completing one step cycle.
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
  TCCR2A = 0; // Normal operation
  TCCR2B = (1<<CS21); // Full speed, 1/8 prescaler
  TIMSK2 |= (1<<TOIE2);      
  
  // Just set the step_timer to something serviceably lazy
  config_step_timer(20000);
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
void config_step_timer(uint32_t microseconds)
{
  uint32_t ticks = microseconds*TICKS_PER_MICROSECOND;
  uint16_t ceiling;
  uint16_t prescaler;
	if (ticks <= 0xffffL) {
		ceiling = ticks;
    prescaler = 0; // prescaler: 0
	} else if (ticks <= 0x7ffffL) {
    ceiling = ticks >> 3;
    prescaler = 1; // prescaler: 8
	} else if (ticks <= 0x3fffffL) {
		ceiling =  ticks >> 6;
    prescaler = 2; // prescaler: 64
	} else if (ticks <= 0xffffffL) {
		ceiling =  (ticks >> 8);
    prescaler = 3; // prescaler: 256
	} else if (ticks <= 0x3ffffffL) {
		ceiling = (ticks >> 10);
    prescaler = 4; // prescaler: 1024
	} else {
	  // Okay, that was slower than we actually go. Just set the slowest speed
		ceiling = 0xffff;
    prescaler = 4;
	}
	// Set prescaler
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | ((prescaler+1)<<CS10);
  // Set ceiling
  OCR1A = ceiling;
}

void st_go_home()
{
  // Todo: Perform the homing cycle
}

void st_stop()
{
	st_flush();
	current_block=NULL;		// Rather brutal. Works!
	
	position[X_AXIS] = actual_position[X_AXIS];
	position[Y_AXIS] = actual_position[Y_AXIS];
	position[Z_AXIS] = actual_position[Z_AXIS];
	
}
