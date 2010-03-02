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
   and Philipp Tiefenbacher. The circle buffer implementation gleaned from the wiring_serial library 
   by David A. Mellis */

#include "stepper.h"
#include "config.h"
#include <math.h>
#include <util/delay.h>
#include "nuts_bolts.h"
#include <avr/interrupt.h>

#include "wiring_serial.h"

#define TICKS_PER_MICROSECOND (F_CPU/1000000)
#define LINE_BUFFER_SIZE 5

struct Line {
  uint32_t steps_x, steps_y, steps_z;
  uint32_t maximum_steps;
  uint32_t iterations;
  uint8_t direction_bits;
  uint32_t rate;
}

volatile uint8_t line_buffer[LINE_BUFFER_SIZE]; // A buffer for step instructions
volatile int line_buffer_head = 0;
volatile int line_buffer_tail = 0;

// Variables used by SIG_OUTPUT_COMPARE1A
uint8_t out_bits;
struct Line *current_line;
uint32_t counter_x, counter_y, counter_z;

uint8_t stepper_mode = STEPPER_MODE_STOPPED;

void config_pace_timer(uint32_t microseconds);

void st_buffer_line(int32_t steps_x, int32_t steps_y, int32_t steps_z, uint32_t rate) {
  // Buffer nothing unless stepping subsystem is running
  if (stepper_mode != STEPPER_MODE_RUNNING) { return; }
  // Calculate the buffer head after we push this byte
	int next_buffer_head = (line_buffer_head + 1) % LINE_BUFFER_SIZE;	
	// If the buffer is full: good! That means we are well ahead of the robot. 
	// Nap until there is room in the buffer.
  while(line_buffer_tail == next_buffer_head) { sleep_mode(); }
  
  // setup line
  struct Line *line = &line_buffer[line_buffer_head];
  line->steps_x = labs(steps_x);
  line->steps_y = labs(steps_y);
  line->steps_z = labs(steps_y);  
  line->maximum_steps = max(line->steps_x, max(line->steps_y, line->steps_z));
  line->iterations = line->maximum_steps;
  line->rate = rate;
  uint8_t direction_bits = 0;
  if (steps_x < 0) { direction_bits |= (1<<X_DIRECTION_BIT); }
  if (steps_y < 0) { direction_bits |= (1<<Y_DIRECTION_BIT); }
  if (steps_z < 0) { direction_bits |= (1<<Z_DIRECTION_BIT); }
  line->direction_bits = direction_bits;
  
  // Move buffer head
  line_buffer_head = next_buffer_head;
}

// This timer interrupt is executed at the pace set with st_buffer_pace. It pops one instruction from
// the line_buffer, executes it. Then it starts timer2 in order to reset the motor port after
// five microseconds.
SIGNAL(SIG_OUTPUT_COMPARE1A)
{
  // Set the direction pins a cuple of nanoseconds before we step the steppers
  STEPPING_PORT = (STEPPING_PORT & ~DIRECTION_MASK) | (out_bits & DIRECTION_MASK);
  // Then pulse the stepping pins
  STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | out_bits;
  // Reset step pulse reset timer
  TCNT2 = -(((STEP_PULSE_MICROSECONDS-4)*TICKS_PER_MICROSECOND)/8);
    
  // If there is no current line, attempt to pop one from the buffer
  if (current_line == NULL) {
    // Anything in the buffer?
    if (line_buffer_head != line_buffer_tail) {
      // Retrieve a new line and get ready to step it
      current_line = &line_buffer[line_buffer_tail]; 
      config_pace_timer(current_line->rate);
      counter_x = -current_line->maximum_steps/2;
      counter_y = counter_x;
      counter_z = counter_x;
      // move the line buffer tail to the next instruction
      line_buffer_tail = (line_buffer_tail + 1) % LINE_BUFFER_SIZE;
    }
  }

  if (current_line != NULL) {
    out_bits = current_line->direction_bits;
    counter_x += current_line->steps_x;
    if (counter_x > 0) {
      out_bits |= (1<<X_STEP_BIT);
      counter_x -= current_line->maximum_steps;
    }
    counter_y += current_line-> steps_y;
    if (counter_y > 0) {
      out_bits |= (1<<Y_STEP_BIT);
      counter_y -= current_line->maximum_steps;
    }
    counter_z += current_line-> steps_z;
    if (counter_z > 0) {
      out_bits |= (1<<Z_STEP_BIT);
      counter_z -= current_line->maximum_steps;
    }
    // If current line is finished, reset pointer 
    current_line->iterations -= 1;
    if (current_line->iterations <= 0) {
      current_line = NULL;
    }
  } else {
    out_bits = 0;
  }
  out_bits ^= STEPPING_INVERT_MASK;
}

// This interrupt is set up by SIG_OUTPUT_COMPARE1A when it sets the motor port bits. It resets
// the motor port after a short period (STEP_PULSE_MICROSECONDS) completing one step cycle.
SIGNAL(SIG_OVERFLOW2)
{
  // reset stepping pins (leave the direction pins)
  STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | (STEPPING_INVERT_MASK & STEP_MASK); 
}

// Initialize and start the stepper motor subsystem
void st_init()
{
	// Configure directions of interface pins
  STEPPING_DDR   |= STEPPING_MASK;
  STEPPING_PORT = (STEPPING_PORT & ~STEPPING_MASK); //| STEPPING_INVERT_MASK;
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
  TIMSK2 = 0; // All interrupts disabled
  
  sei();
  
	// start off with a mellow pace
  config_pace_timer(20000);
}

// Block until all buffered steps are executed
void st_synchronize()
{
  if (stepper_mode == STEPPER_MODE_RUNNING) {
    while(line_buffer_tail != line_buffer_head) { sleep_mode(); }    
  } else {
    st_flush();
  }
}

// Cancel all pending steps
void st_flush()
{
  cli();
  line_buffer_tail = line_buffer_head;
  current_line = NULL;
  sei();
}

// Start the stepper subsystem
void st_start()
{
  // Enable timer interrupts
	TIMSK1 |= (1<<OCIE1A);
  TIMSK2 |= (1<<TOIE2);      
  // set enable pin   
  STEPPERS_ENABLE_PORT |= 1<<STEPPERS_ENABLE_BIT;
  stepper_mode = STEPPER_MODE_RUNNING;
}

// Execute all buffered steps, then stop the stepper subsystem
inline void st_stop()
{
  // flush pending operations
  st_synchronize();
  // disable timer interrupts
	TIMSK1 &= ~(1<<OCIE1A);
  TIMSK2 &= ~(1<<TOIE2);           
  // reset enable pin
  STEPPERS_ENABLE_PORT &= ~(1<<STEPPERS_ENABLE_BIT);
  stepper_mode = STEPPER_MODE_STOPPED;
}

// Returns a bitmask with the stepper bit for the given axis set
uint8_t st_bit_for_stepper(int axis) {
  switch(axis) {
    case X_AXIS: return(1<<X_STEP_BIT);
    case Y_AXIS: return(1<<Y_STEP_BIT);
    case Z_AXIS: return(1<<Z_STEP_BIT);
  }
  return(0);
}

// Configures the prescaler and ceiling of timer 1 to produce the given pace as accurately as possible.
void config_pace_timer(uint32_t microseconds)
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
  current_pace = microseconds;
}

int check_limit_switches()
{
  // Dual read as crude debounce
  return((LIMIT_PORT & LIMIT_MASK) | (LIMIT_PORT & LIMIT_MASK));
}

int check_limit_switch(int axis)
{
  uint8_t mask = 0;
  switch (axis) {
    case X_AXIS: mask = 1<<X_LIMIT_BIT; break;
    case Y_AXIS: mask = 1<<Y_LIMIT_BIT; break;
    case Z_AXIS: mask = 1<<Z_LIMIT_BIT; break;
  }
  return((LIMIT_PORT&mask) || (LIMIT_PORT&mask));    
}

void st_go_home()
{
  // Todo: Perform the homing cycle
}

// Convert from millimeters to step-counts along the designated axis
int32_t st_millimeters_to_steps(double millimeters, int axis) {
  switch(axis) {
    case X_AXIS: return(round(millimeters*X_STEPS_PER_MM));
    case Y_AXIS: return(round(millimeters*Y_STEPS_PER_MM));
    case Z_AXIS: return(round(millimeters*Z_STEPS_PER_MM));
  }
  return(0);
}
