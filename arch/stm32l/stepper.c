/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Sungeun K. Jeon
  
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

#include "dev_misc.h"
#include "stepper.h"
#include "config.h"
#include "settings.h"
#include <math.h>
#include <stdlib.h>
#include "nuts_bolts.h"
#include "planner.h"
#include "limits.h"

#include <stm32f10x.h>
#include <stm32f10x_conf.h>

#define STEPPER_X_A1 	GPIO_Pin_0	
#define STEPPER_X_A2 	GPIO_Pin_1	
#define STEPPER_X_B1 	GPIO_Pin_2	
#define STEPPER_X_B2 	GPIO_Pin_3

#define STEPPER_Y_A1    GPIO_Pin_4
#define STEPPER_Y_A2 	GPIO_Pin_5
#define STEPPER_Y_B1 	GPIO_Pin_6
#define STEPPER_Y_B2 	GPIO_Pin_7

#define STEPPER_Z_A1    GPIO_Pin_8
#define STEPPER_Z_A2 	GPIO_Pin_9
#define STEPPER_Z_B1 	GPIO_Pin_10 
#define STEPPER_Z_B2 	GPIO_Pin_11

#define STEPPER_X_ENABLE GPIO_Pin_8
#define STEPPER_Y_ENABLE GPIO_Pin_9

#define STEPPER_ALL_PINS (STEPPER_X_A1|STEPPER_X_A2|STEPPER_X_B1|STEPPER_X_B2|STEPPER_Y_A1|STEPPER_Y_A2|STEPPER_Y_B1|STEPPER_Y_B2|STEPPER_X_ENABLE|STEPPER_Y_ENABLE)

#define STEPPER_PORT 	GPIOC

#define TICKS_PER_MICROSECOND (F_CPU/1000000)
#define CYCLES_PER_ACCELERATION_TICK ((TICKS_PER_MICROSECOND*1000000)/ACCELERATION_TICKS_PER_SECOND)

#define DIR_FORWARD	1
#define DIR_BACKWARD	0

// Stepper state variable. Contains running data and trapezoid variables.
typedef struct {
  // Used by the bresenham line algorithm
  int32_t counter_x,        // Counter variables for the bresenham line tracer
          counter_y, 
          counter_z;
  uint32_t event_count;
  uint32_t step_events_completed;  // The number of step events left in current motion

  // Used by the trapezoid generator
  uint32_t cycles_per_step_event;        // The number of machine cycles between each step event
  uint32_t trapezoid_tick_cycle_counter; // The cycles since last trapezoid_tick. Used to generate ticks at a steady
                                              // pace without allocating a separate timer
  uint32_t trapezoid_adjusted_rate;      // The current rate of step_events according to the trapezoid generator
  uint32_t min_safe_rate;  // Minimum safe rate for full deceleration rate reduction step. Otherwise halves step_rate.
} stepper_t;

static stepper_t st;
static block_t *current_block;  // A pointer to the block currently being traced

// Used by the stepper driver interrupt
static uint8_t step_pulse_time; // Step pulse reset time after step rise
static uint8_t out_bits;        // The next stepping-bits to be output

static int do_int = 0;

void do_full_step(int direction, int axis);
void do_half_step(int direction, int axis);

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
//  The slope of acceleration is always +/- block->rate_delta and is applied at a constant rate following the midpoint rule
//  by the trapezoid generator, which is called ACCELERATION_TICKS_PER_SECOND times per second.

static void set_step_events_per_minute(uint32_t steps_per_minute);

// Stepper state initialization
static void st_wake_up() 
{
  // Initialize stepper output bits
  out_bits = (0) ^ (settings.invert_mask); 
  // Set step pulse time. Ad hoc computation from oscilloscope.
  step_pulse_time = -(((settings.pulse_microseconds-2)*TICKS_PER_MICROSECOND) >> 3);
  // Enable steppers by resetting the stepper disable port
  //STEPPERS_DISABLE_PORT &= ~(1<<STEPPERS_DISABLE_BIT);
  // Enable stepper driver interrupt
	do_int = 1; 
}

// Stepper shutdown
void st_go_idle() 
{
  // Disable stepper driver interrupt
 	do_int = 0;
  // Force stepper dwell to lock axes for a defined amount of time to ensure the axes come to a complete
  // stop and not drift from residual inertial forces at the end of the last movement.
  #ifdef STEPPER_IDLE_LOCK_TIME
  //  _delay_ms(STEPPER_IDLE_LOCK_TIME);   
  #endif
  // Disable steppers by setting stepper disable
 // STEPPERS_DISABLE_PORT |= (1<<STEPPERS_DISABLE_BIT);
}

// This function determines an acceleration velocity change every CYCLES_PER_ACCELERATION_TICK by
// keeping track of the number of elapsed cycles during a de/ac-celeration. The code assumes that 
// step_events occur significantly more often than the acceleration velocity iterations.
static uint8_t iterate_trapezoid_cycle_counter() 
{
  st.trapezoid_tick_cycle_counter += st.cycles_per_step_event;  
  if(st.trapezoid_tick_cycle_counter > CYCLES_PER_ACCELERATION_TICK) {
    st.trapezoid_tick_cycle_counter -= CYCLES_PER_ACCELERATION_TICK;
    return(true);
  } else {
    return(false);
  }
}          

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse of Grbl. It is executed at the rate set with
// config_step_timer. It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
// It is supported by The Stepper Port Reset Interrupt which it uses to reset the stepper port after each pulse. 
// The bresenham line tracer algorithm controls all three stepper outputs simultaneously with these two interrupts.
// NOTE: ISR_NOBLOCK allows SIG_OVERFLOW2 to trigger on-time regardless of time in this handler.

// TODO: ISR_NOBLOCK is the same as the old SIGNAL with sei() method, but is optimizable by the compiler. On
// an oscilloscope there is a weird hitch in the step pulse during high load operation. Very infrequent, but
// when this does happen most of the time the pulse falling edge is randomly delayed by 20%-50% of the total 
// intended pulse time, but sometimes it pulses less than 3usec. The former likely caused by the serial 
// interrupt doing its thing, not that big of a deal, but the latter cause is unknown and worrisome. Need
// to track down what is causing this problem. Functionally, this shouldn't cause any noticeable issues
// as long as stepper drivers have a pulse minimum of 1usec or so (Pololu and any Allegro IC are ok).
// ** This seems to be an inherent issue that dates all the way back to Simen's v0.6b or earlier. **

void tim2_isr() __attribute__ ((optimize(0)));// this interrupt breaks if it is optimized!!!!!!!!!!!!
void tim2_isr()
{
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);	//Interrupt Flag von TIM2 Löschen

	toggle_led();	

	if(!do_int)
		return;

	if(out_bits & (1<<X_STEP_BIT)) {
		if(out_bits & (1<<X_DIRECTION_BIT)) {
			do_full_step(DIR_FORWARD, X_AXIS);
		} else {
			do_full_step(DIR_BACKWARD, X_AXIS);
		}
	}	
	if(out_bits & (1<<Y_STEP_BIT)) {
		if(out_bits & (1<<Y_DIRECTION_BIT)) {
			do_full_step(DIR_FORWARD, Y_AXIS);
		} else {
			do_full_step(DIR_BACKWARD, Y_AXIS);
		}	
	}
	if(out_bits & (1<<Z_STEP_BIT)) {
		if(out_bits & (1<<Z_DIRECTION_BIT)) {
			do_full_step(DIR_FORWARD, Y_AXIS);
		} else {
			do_full_step(DIR_BACKWARD, Y_AXIS);
		}	
	}
  
  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
    // Anything in the buffer? If so, initialize next motion.
    current_block = plan_get_current_block();
    if (current_block != NULL) {
      if (!sys.feed_hold) { 
        // During feed hold, do not update rate and trap counter. Keep decelerating.
        st.trapezoid_adjusted_rate = current_block->initial_rate;
        set_step_events_per_minute(st.trapezoid_adjusted_rate); // Initialize cycles_per_step_event
        st.trapezoid_tick_cycle_counter = CYCLES_PER_ACCELERATION_TICK/2; // Start halfway for midpoint rule.
      }
      st.min_safe_rate = current_block->rate_delta + (current_block->rate_delta >> 1); // 1.5 x rate_delta
      st.counter_x = -(current_block->step_event_count >> 1);
      st.counter_y = st.counter_x;
      st.counter_z = st.counter_x;
      st.event_count = current_block->step_event_count;
      st.step_events_completed = 0;     
    } else {
      st_go_idle();
      sys.cycle_start = false;
      bit_true(sys.execute,EXEC_CYCLE_STOP); // Flag main program for cycle end
    }    
  } 

  if (current_block != NULL) {
    // Execute step displacement profile by bresenham line algorithm
    out_bits = current_block->direction_bits;
    st.counter_x += current_block->steps_x;
    if (st.counter_x > 0) {
      out_bits |= (1<<X_STEP_BIT);
      st.counter_x -= st.event_count;
      if (out_bits & (1<<X_DIRECTION_BIT)) { sys.position[X_AXIS]--; }
      else { sys.position[X_AXIS]++; }
    }
    st.counter_y += current_block->steps_y;
    if (st.counter_y > 0) {
      out_bits |= (1<<Y_STEP_BIT);
      st.counter_y -= st.event_count;
      if (out_bits & (1<<Y_DIRECTION_BIT)) { sys.position[Y_AXIS]--; }
      else { sys.position[Y_AXIS]++; }
    }
    st.counter_z += current_block->steps_z;
    if (st.counter_z > 0) {
      out_bits |= (1<<Z_STEP_BIT);
      st.counter_z -= st.event_count;
      if (out_bits & (1<<Z_DIRECTION_BIT)) { sys.position[Z_AXIS]--; }
      else { sys.position[Z_AXIS]++; }
    }
    
    st.step_events_completed++; // Iterate step events

    // While in block steps, check for de/ac-celeration events and execute them accordingly.
    if (st.step_events_completed < current_block->step_event_count) {
      if (sys.feed_hold) {
        // Check for and execute feed hold by enforcing a steady deceleration from the moment of 
        // execution. The rate of deceleration is limited by rate_delta and will never decelerate
        // faster or slower than in normal operation. If the distance required for the feed hold 
        // deceleration spans more than one block, the initial rate of the following blocks are not
        // updated and deceleration is continued according to their corresponding rate_delta.
        // NOTE: The trapezoid tick cycle counter is not updated intentionally. This ensures that 
        // the deceleration is smooth regardless of where the feed hold is initiated and if the
        // deceleration distance spans multiple blocks.
        if ( iterate_trapezoid_cycle_counter() ) {                    
          // If deceleration complete, set system flags and shutdown steppers.
          if (st.trapezoid_adjusted_rate <= current_block->rate_delta) {
            // Just go idle. Do not NULL current block. The bresenham algorithm variables must
            // remain intact to ensure the stepper path is exactly the same. Feed hold is still
            // active and is released after the buffer has been reinitialized.
            st_go_idle();
            sys.cycle_start = false;
            bit_true(sys.execute,EXEC_CYCLE_STOP); // Flag main program that feed hold is complete.
          } else {
            st.trapezoid_adjusted_rate -= current_block->rate_delta;
            set_step_events_per_minute(st.trapezoid_adjusted_rate);
          }      
        }
        
      } else {
        // The trapezoid generator always checks step event location to ensure de/ac-celerations are 
        // executed and terminated at exactly the right time. This helps prevent over/under-shooting
        // the target position and speed. 
        // NOTE: By increasing the ACCELERATION_TICKS_PER_SECOND in config.h, the resolution of the 
        // discrete velocity changes increase and accuracy can increase as well to a point. Numerical 
        // round-off errors can effect this, if set too high. This is important to note if a user has 
        // very high acceleration and/or feedrate requirements for their machine.
        if (st.step_events_completed < current_block->accelerate_until) {
          // Iterate cycle counter and check if speeds need to be increased.
          if ( iterate_trapezoid_cycle_counter() ) {
            st.trapezoid_adjusted_rate += current_block->rate_delta;
            if (st.trapezoid_adjusted_rate >= current_block->nominal_rate) {
              // Reached nominal rate a little early. Cruise at nominal rate until decelerate_after.
              st.trapezoid_adjusted_rate = current_block->nominal_rate;
            }
            set_step_events_per_minute(st.trapezoid_adjusted_rate);
          }
        } else if (st.step_events_completed >= current_block->decelerate_after) {
          // Reset trapezoid tick cycle counter to make sure that the deceleration is performed the
          // same every time. Reset to CYCLES_PER_ACCELERATION_TICK/2 to follow the midpoint rule for
          // an accurate approximation of the deceleration curve.
          if (st.step_events_completed == current_block-> decelerate_after) {
            st.trapezoid_tick_cycle_counter = CYCLES_PER_ACCELERATION_TICK/2;
          } else {
            // Iterate cycle counter and check if speeds need to be reduced.
            if ( iterate_trapezoid_cycle_counter() ) {  
              // NOTE: We will only do a full speed reduction if the result is more than the minimum safe 
              // rate, initialized in trapezoid reset as 1.5 x rate_delta. Otherwise, reduce the speed by
              // half increments until finished. The half increments are guaranteed not to exceed the 
              // CNC acceleration limits, because they will never be greater than rate_delta. This catches
              // small errors that might leave steps hanging after the last trapezoid tick or a very slow
              // step rate at the end of a full stop deceleration in certain situations. The half rate 
              // reductions should only be called once or twice per block and create a nice smooth 
              // end deceleration.
              if (st.trapezoid_adjusted_rate > st.min_safe_rate) {
                st.trapezoid_adjusted_rate -= current_block->rate_delta;
              } else {
                st.trapezoid_adjusted_rate >>= 1; // Bit shift divide by 2
              }
              if (st.trapezoid_adjusted_rate < current_block->final_rate) {
                // Reached final rate a little early. Cruise to end of block at final rate.
                st.trapezoid_adjusted_rate = current_block->final_rate;
              }
              set_step_events_per_minute(st.trapezoid_adjusted_rate);
            }
          }
        } else {
          // No accelerations. Make sure we cruise exactly at the nominal rate.
          if (st.trapezoid_adjusted_rate != current_block->nominal_rate) {
            st.trapezoid_adjusted_rate = current_block->nominal_rate;
            set_step_events_per_minute(st.trapezoid_adjusted_rate);
          }
        }
      }            
    } else {   
      // If current block is finished, reset pointer 
      current_block = NULL;
      plan_discard_current_block();
    }
  }
  out_bits ^= settings.invert_mask;  // Apply stepper invert mask    
}

// Reset and clear stepper subsystem variables
void st_reset()
{
  memset(&st, 0, sizeof(st));
  set_step_events_per_minute(MINIMUM_STEPS_PER_MINUTE);
  current_block = NULL;
}

// Initialize and start the stepper motor subsystem
void st_init()
{

	GPIO_InitTypeDef gpio_init;

	gpio_init.GPIO_Pin = STEPPER_ALL_PINS;	
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(STEPPER_PORT, &gpio_init);
	
	STEPPER_PORT->BRR = STEPPER_ALL_PINS;

	TIM_TimeBaseInitTypeDef timer_settings;
	/* Time base configuration */
	timer_settings.TIM_Period = MINIMUM_STEPS_PER_MINUTE;
	timer_settings.TIM_Prescaler = 1-1;
	timer_settings.TIM_ClockDivision = TIM_CKD_DIV1;
	timer_settings.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &timer_settings);

	/* Clear TIM2 update pending flag */
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);

	/* TIM IT enable */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	TIM_ClearFlag(TIM2, TIM_FLAG_Update);

	TIM_Cmd(TIM2, ENABLE);
	
	// Start in the idle state
	st_go_idle();
}

static void set_step_events_per_minute(uint32_t steps_per_minute) 
{
  if (steps_per_minute < MINIMUM_STEPS_PER_MINUTE) { steps_per_minute = MINIMUM_STEPS_PER_MINUTE; }
  //(1/(steps_per_minute*60))/(1/(F_CPU/PRESCALER)) 
  st.cycles_per_step_event = ((24000000/60)/(steps_per_minute));
  TIM2->ARR =  st.cycles_per_step_event;
}

// Planner external interface to start stepper interrupt and execute the blocks in queue. Called
// by the main program functions: planner auto-start and run-time command execution.
void st_cycle_start() 
{
  if (!sys.cycle_start) {
    if (!sys.feed_hold) {
      sys.cycle_start = true;
      st_wake_up();
    }
  }
}

// Execute a feed hold with deceleration, only during cycle. Called by main program.
void st_feed_hold() 
{
  if (!sys.feed_hold) {
    if (sys.cycle_start) {
      sys.auto_start = false; // Disable planner auto start upon feed hold.
      sys.feed_hold = true;
    }
  }
}

// Reinitializes the cycle plan and stepper system after a feed hold for a resume. Called by 
// runtime command execution in the main program, ensuring that the planner re-plans safely.
// NOTE: Bresenham algorithm variables are still maintained through both the planner and stepper
// cycle reinitializations. The stepper path should continue exactly as if nothing has happened.
// Only the planner de/ac-celerations profiles and stepper rates have been updated.
void st_cycle_reinitialize()
{
  if (current_block != NULL) {
    // Replan buffer from the feed hold stop location.
    plan_cycle_reinitialize(current_block->step_event_count - st.step_events_completed);
    // Update initial rate and timers after feed hold.
    st.trapezoid_adjusted_rate = 0; // Resumes from rest
    set_step_events_per_minute(st.trapezoid_adjusted_rate);
    st.trapezoid_tick_cycle_counter = CYCLES_PER_ACCELERATION_TICK/2; // Start halfway for midpoint rule.
    st.step_events_completed = 0;
  }
  sys.feed_hold = false; // Release feed hold. Cycle is ready to re-start.
}

const int stepper_pins[3][4] = {
		{STEPPER_X_A1, STEPPER_X_A2, STEPPER_X_B1, STEPPER_X_B2},
		{STEPPER_Y_A1, STEPPER_Y_A2, STEPPER_Y_B1, STEPPER_Y_B2},
		{STEPPER_Z_A1, STEPPER_Z_A2, STEPPER_Z_B1, STEPPER_Z_B2},
		};

void do_half_step(int direction, int axis) 
{
	static int crrnt_step[3] = {0,0,0};
	
	if(direction == DIR_FORWARD) {
		crrnt_step[axis] ++;
		if (crrnt_step[axis] >= 8)
			crrnt_step[axis] = 0;
	}
	else{
		if(crrnt_step[axis] == 0)
			crrnt_step[axis] = 8;
		crrnt_step[axis] --;			
	}

		//BRR is clear and BSRR is set half step is probably broken.. I dont use it.
	switch(crrnt_step[axis]) {
		case 0:
			STEPPER_PORT->BRR = stepper_pins[axis][0];
			STEPPER_PORT->BSRR = stepper_pins[axis][3];
			break;
		case 1:
			STEPPER_PORT->BRR =  stepper_pins[axis][0] | stepper_pins[axis][2];
; 	
			break;
		case 2:
			STEPPER_PORT->BRR = stepper_pins[axis][2];
			STEPPER_PORT->BSRR = stepper_pins[axis][0];
			break;	
		case 3:
			STEPPER_PORT->BRR =  stepper_pins[axis][1] | stepper_pins[axis][2];
			break;
		case 4:		
			STEPPER_PORT->BRR = stepper_pins[axis][1];
			STEPPER_PORT->BSRR = stepper_pins[axis][2];
			break;	
		case 5: 
			STEPPER_PORT->BRR =  stepper_pins[axis][1] | stepper_pins[axis][3];
			break;
		case 6:	
			STEPPER_PORT->BRR = stepper_pins[axis][3];
			STEPPER_PORT->BSRR = stepper_pins[axis][1];
			break;	
		case 7:
			STEPPER_PORT->BRR =  stepper_pins[axis][0] | stepper_pins[axis][3];
			break;
	}
}

void do_full_step(int direction, int axis) 
{
	static unsigned int crrnt_step[3] = {0,0,0};
	
	if(direction == DIR_FORWARD) {
		crrnt_step[axis] ++;
		if (crrnt_step[axis] >= 4)
			crrnt_step[axis] = 0;
	}
	else{
		if(crrnt_step[axis] == 0)
			crrnt_step[axis] = 4;
		crrnt_step[axis] --;			
	}
	
	switch(crrnt_step[axis]) {
		case 0:
			STEPPER_PORT->BRR = stepper_pins[axis][0] | stepper_pins[axis][2];
			STEPPER_PORT->BSRR = stepper_pins[axis][3] | stepper_pins[axis][1];
			break;
		case 1:			
			STEPPER_PORT->BRR = stepper_pins[axis][1] | stepper_pins[axis][2];
			STEPPER_PORT->BSRR = stepper_pins[axis][0] | stepper_pins[axis][3];
			break;
		case 2:				
			STEPPER_PORT->BRR = stepper_pins[axis][1] | stepper_pins[axis][3];
			STEPPER_PORT->BSRR = stepper_pins[axis][2] | stepper_pins[axis][0];
			break;
		case 3:	
			STEPPER_PORT->BRR = stepper_pins[axis][0] | stepper_pins[axis][3];
			STEPPER_PORT->BSRR = stepper_pins[axis][1] | stepper_pins[axis][2];
			break;
	}
}

