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

#include "stepper.h"
#include "config.h"
#include "settings.h"
#include <math.h>
#include <stdlib.h>
#include "nuts_bolts.h"
#include "planner.h"
#include "limits.h"
#include <pthread.h>

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>

timer_t timerid;

#define DIR_FORWARD 1
#define DIR_BACKWARD 0

int simul_running = 1;
void do_step(int direction, int axis);
extern int cnc_head_x;
extern int cnc_head_y;

// Some useful constants
#define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits
#define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits
#define STEPPING_MASK (STEP_MASK | DIRECTION_MASK) // All stepping-related bits (step/direction)

#define TICKS_PER_MICROSECOND (F_CPU/1000000)
#define CYCLES_PER_ACCELERATION_TICK ((TICKS_PER_MICROSECOND*1000000)/ACCELERATION_TICKS_PER_SECOND)

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
static volatile uint8_t busy;   // True when SIG_OUTPUT_COMPARE1A is being serviced. Used to avoid retriggering that handler.
int volatile do_int = 0;
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
	//  STEPPERS_DISABLE_PORT &= ~(1<<STEPPERS_DISABLE_BIT);
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
	//   _delay_ms(STEPPER_IDLE_LOCK_TIME);   
#endif
	// Disable steppers by setting stepper disable
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
static void handler(int sig, siginfo_t *si, void *uc)
{        
	if(do_int ){
		if (busy) { return; } // The busy-flag is used to avoid reentering this interrupt
		if(out_bits & (1<<X_STEP_BIT)) {
			if(out_bits & (1<<X_DIRECTION_BIT)) {
				do_step(DIR_FORWARD, X_AXIS);
			} else {
				do_step(DIR_BACKWARD, X_AXIS);
			}
		}	
		if(out_bits & (1<<Y_STEP_BIT)) {
			if(out_bits & (1<<Y_DIRECTION_BIT)) {
				do_step(DIR_FORWARD, Y_AXIS);
			} else {
				do_step(DIR_BACKWARD, Y_AXIS);
			}	
		}
		if(out_bits & (1<<Z_STEP_BIT)) {
			if(out_bits & (1<<Z_DIRECTION_BIT)) {
				do_step(DIR_FORWARD, Z_AXIS);
			} else {
				do_step(DIR_BACKWARD, Z_AXIS);
			}
		}
		//  TCNT2 = step_pulse_time; // Reload timer counter
		//  TCCR2B = (1<<CS21); // Begin timer2. Full speed, 1/8 prescaler

		busy = true;
		// Re-enable interrupts to allow ISR_TIMER2_OVERFLOW to trigger on-time and allow serial communications
		// regardless of time in this handler. The following code prepares the stepper driver for the next
		// step interrupt compare and will always finish before returning to the main program.
		//sei();

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
		busy = false;
	}

	struct itimerspec its;
	long long freq_nanosecs = st.cycles_per_step_event*1000;
	its.it_value.tv_sec = freq_nanosecs / 1000000000;
	its.it_value.tv_nsec = freq_nanosecs % 1000000000;
	its.it_interval.tv_sec = its.it_value.tv_sec;
	its.it_interval.tv_nsec = its.it_value.tv_nsec;

	if (timer_settime(timerid, 0, &its, NULL) == -1)
		printf("timer_settime error\n");
}	

/*
// This interrupt is set up by ISR_TIMER1_COMPAREA when it sets the motor port bits. It resets
// the motor port after a short period (settings.pulse_microseconds) completing one step cycle.
// TODO: It is possible for the serial interrupts to delay this interrupt by a few microseconds, if
// they execute right before this interrupt. Not a big deal, but could use some TLC at some point.
ISR(TIMER2_OVF_vect)
{
// Reset stepping pins (leave the direction pins)
STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | (settings.invert_mask & STEP_MASK); 
TCCR2B = 0; // Disable Timer2 to prevent re-entering this interrupt when it's not needed. 
}
*/

// Reset and clear stepper subsystem variables
void st_reset()
{
	memset(&st, 0, sizeof(st));
	set_step_events_per_minute(MINIMUM_STEPS_PER_MINUTE);
	current_block = NULL;
	busy = false;
}

// Initialize and start the stepper motor subsystem
void st_init()
{
	// Start in the idle state
	st_go_idle();
	set_step_events_per_minute(1200);
#define SIG SIGRTMIN

	struct sigevent sev;
	struct itimerspec its;
	long long freq_nanosecs;
	struct sigaction sa;
	sigset_t mask;

	/* Establish handler for timer signal */

	printf("Establishing handler for signal %d\n", SIG);
	sa.sa_flags = SA_SIGINFO;
	sa.sa_sigaction = handler;
	sigemptyset(&sa.sa_mask);
	if (sigaction(SIG, &sa, NULL) == -1)
		printf("error: sigaction\n");

	// Block timer signal temporarily
	printf("Blocking signal %d\n", SIG);
	sigemptyset(&mask);
	sigaddset(&mask, SIG);
	if (sigprocmask(SIG_SETMASK, &mask, NULL) == -1)
		printf("error: sigprocmask\n");


	/* Create the timer */

	sev.sigev_notify = SIGEV_SIGNAL;
	sev.sigev_signo = SIG;
	sev.sigev_value.sival_ptr = &timerid;
	if (timer_create(CLOCK_REALTIME, &sev, &timerid) == -1)
		printf("error: timer_create\n");

	printf("timer ID is 0x%lx\n", (long) timerid);

	/* Start the timer */

	freq_nanosecs = st.cycles_per_step_event*1000;
	its.it_value.tv_sec = freq_nanosecs / 1000000000;
	its.it_value.tv_nsec = freq_nanosecs % 1000000000;
	its.it_interval.tv_sec = its.it_value.tv_sec;
	its.it_interval.tv_nsec = its.it_value.tv_nsec;

	if (timer_settime(timerid, 0, &its, NULL) == -1)
		printf("timer_settime error\n");

	/* Sleep for a while; meanwhile, the timer may expire
	   multiple times */

	/* Unlock the timer signal, so that timer notification
	   can be delivered */

	printf("Unblocking signal %d\n", SIG);
	if (sigprocmask(SIG_UNBLOCK, &mask, NULL) == -1)
		printf("error: sigprocmask\n");

	init_graphics();
}

static void set_step_events_per_minute(uint32_t steps_per_minute) 
{
	if (steps_per_minute < MINIMUM_STEPS_PER_MINUTE) { steps_per_minute = MINIMUM_STEPS_PER_MINUTE; }
	//microseconds//
	st.cycles_per_step_event = (1000000/steps_per_minute*60);  

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


void do_step(int direction, int axis)
{
	int step = 0;
	if(direction == DIR_FORWARD) {
		step = -4;///DEFAULT_X_STEPS_PER_MM;
	}
	else {
		step = 4;///DEFAULT_y_STEPS_PER_MM;
	}
	switch(axis) {
		case X_AXIS:
			cnc_head_x += step;
			break;
		case Y_AXIS:
			cnc_head_y += step;
			break;
		case Z_AXIS:
			break;
	}
}

