/*
  motion_control.c - cartesian robot controller.
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

#include <avr/io.h>
#include "config.h"
#include "motion_control.h"
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#include "nuts_bolts.h"
#include "stepper.h"
#include "gcode.h"
#include "wiring_serial.h"
#include <avr/pgmspace.h>   // contains PSTR definition


int32_t position[3];    // The current position of the tool in absolute steps
extern int32_t actual_position[3];
volatile char mc_running=0;
int32_t acting_line_number=0;

void mc_init()
{
  clear_vector(position);
}

void mc_dwell(uint32_t milliseconds, int32_t line_number) 
{
/*  st_synchronize();
  _delay_ms(milliseconds);
  acting_line_number=line_number;
*/

  st_buffer_delay(milliseconds);
  acting_line_number=line_number;

}


void mc_halt(int32_t line_number)
{
  st_buffer_delay(0);
  acting_line_number=line_number;
}

// Execute linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
// unless invert_feed_rate is true. Then the feed_rate means that the motion should be completed in
// 1/feed_rate minutes.
void mc_line(double x, double y, double z, float feed_rate, int invert_feed_rate, int32_t line_number)
{
  uint8_t axis; // loop variable
  int32_t target[3]; // The target position in absolute steps
  int32_t steps[3]; // The target line in relative steps
  
  target[X_AXIS] = lround(x*settings.steps_per_mm[0]);
  target[Y_AXIS] = lround(y*settings.steps_per_mm[1]);
  target[Z_AXIS] = lround(z*settings.steps_per_mm[2]); 

  for(axis = X_AXIS; axis <= Z_AXIS; axis++) {
    steps[axis] = target[axis]-position[axis];
  }
  
	if (invert_feed_rate) {
    	st_buffer_block(steps[X_AXIS], steps[Y_AXIS], steps[Z_AXIS], 
                        position[X_AXIS], position[Y_AXIS], position[Z_AXIS], 
                        lround(ONE_MINUTE_OF_MICROSECONDS/feed_rate),
                        line_number);
	} else {
  	// Ask old Phytagoras to estimate how many mm our next move is going to take us
  	double millimeters_of_travel = sqrt(
  	  square(steps[X_AXIS]/settings.steps_per_mm[0]) + 
  	  square(steps[Y_AXIS]/settings.steps_per_mm[1]) + 
  	  square(steps[Z_AXIS]/settings.steps_per_mm[2]));
		st_buffer_block(steps[X_AXIS], steps[Y_AXIS], steps[Z_AXIS],
                         position[X_AXIS], position[Y_AXIS], position[Z_AXIS], 
                         lround((millimeters_of_travel/feed_rate)*1000000),
                         line_number);
	}
	memcpy(position, target, sizeof(target)); // position[] = target[] 
}

void mc_reposition(double x, double y, double z, int32_t line_number)
{
  int32_t target[3]; // The target position in absolute steps
  
  target[X_AXIS] = lround(x*settings.steps_per_mm[0]);
  target[Y_AXIS] = lround(y*settings.steps_per_mm[1]);
  target[Z_AXIS] = lround(z*settings.steps_per_mm[2]); 
  
  memcpy(position, target, sizeof(target)); // position[] = target[] 
  memcpy(actual_position, target, sizeof(target));	// Only really necessary for display - actual
  													// position gets updated at the start of the 
  													// next move, but until then the display would
  													// be wrong.
  acting_line_number=line_number;

}

// Execute an arc. theta == start angle, angular_travel == number of radians to go along the arc,
// positive angular_travel means clockwise, negative means counterclockwise. Radius == the radius of the
// circle in millimeters. axis_1 and axis_2 selects the circle plane in tool space. Stick the remaining
// axis in axis_l which will be the axis for linear travel if you are tracing a helical motion.

struct arc_to_lineS {    // Contains the low level representation for an arc 
            double target[3];    // in a way that can be converted to lines
            double theta;
            double theta_per_segment;
            double linear_per_segment;
            double center_x;
            double center_y;
            double radius;
            double feed_rate;
            double end[3];
            int axis_1;
            int axis_2;
            int axis_linear;
            int invert_feed_rate; 
            int32_t line_number;
            uint16_t segments;
            uint16_t i;
   volatile uint8_t active;
} arc; 


void mc_continue_arc(){
    if (st_buffer_full()) return;
    while ((arc.active)&&(!st_buffer_full())){
        if (arc.i <arc.segments){
			arc.target[arc.axis_linear] += arc.linear_per_segment;
			arc.theta += arc.theta_per_segment;
			arc.target[arc.axis_1] = arc.center_x+sin(arc.theta)*arc.radius;
			arc.target[arc.axis_2] = arc.center_y+cos(arc.theta)*arc.radius;
			mc_line(arc.target[X_AXIS], 
					arc.target[Y_AXIS], 
					arc.target[Z_AXIS], 
					arc.feed_rate, arc.invert_feed_rate, arc.line_number);
		} else {
			mc_line(arc.end[X_AXIS], 
					arc.end[Y_AXIS], 
					arc.end[Z_AXIS], 
					arc.feed_rate, arc.invert_feed_rate, arc.line_number);
		}
		arc.i +=1;
		arc.active = (arc.i<=arc.segments);

    }
}


// The arc is approximated by generating a huge number of tiny, linear segments. 
// The length of each segment is configured in config.h by setting MM_PER_ARC_SEGMENT.  
void mc_arc(double theta, double angular_travel, double radius, 
            double linear_travel, 
            int axis_1, int axis_2, int axis_linear, 
            double target_x, double target_y, double target_z,
            double feed_rate, int invert_feed_rate, int32_t line_number)
{
  double millimeters_of_travel = hypot(angular_travel*radius, labs(linear_travel));
  if (millimeters_of_travel == 0.0) { 
      printPgmString(PSTR("\r\nArc length is 0, returning...\r\n")); 
      return; 
  }
    
  arc.end[X_AXIS] = target_x;
  arc.end[Y_AXIS] = target_y;
  arc.end[Z_AXIS] = target_z;
  arc.axis_linear = axis_linear;
  arc.axis_1 = axis_1;
  arc.axis_2 = axis_2;
  arc.line_number = line_number;
    arc.theta = theta;
  arc.radius = radius;
  arc.segments = ceil(millimeters_of_travel/settings.mm_per_arc_segment);
  // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
  // by a number of discrete segments. The inverse feed_rate should be correct for the sum of 
  // all segments.
  if (invert_feed_rate) { feed_rate *= arc.segments; }
  arc.feed_rate = feed_rate;
  arc.invert_feed_rate = invert_feed_rate;
  // The angular motion for each segment
  arc.theta_per_segment = angular_travel/arc.segments;
  // The linear motion for each segment
  arc.linear_per_segment = linear_travel/arc.segments;
  // Compute the center of this circle
  arc.center_x = (position[axis_1]/settings.steps_per_mm[axis_1])-sin(theta)*radius;
  arc.center_y = (position[axis_2]/settings.steps_per_mm[axis_2])-cos(theta)*radius;
  arc.target[axis_linear] = position[axis_linear]/settings.steps_per_mm[Z_AXIS];
    
  arc.i=0;
  arc.active=1;
  mc_continue_arc();					// Start drawing arc
}

// Are we busy drawing an arc?
uint8_t mc_in_arc(){
   return arc.active;
}

void mc_go_home(int32_t line_number)
{
  st_go_home();
  clear_vector(position); // By definition this is location [0, 0, 0]
  acting_line_number=line_number;
}

void mc_stop()
{
	st_stop();              // Stops machine, clears step buffer
                                // sets target position to actual
        set_gcPosition();        // Sets position in g-code interpreter
                                // to current position. Otherwise
                                // next command in g-code contains
                                // wrong position info
	acting_line_number=-1;
}

/*
void mc_continue(int line_number)
{	
	st_continue();
}

void mc_next(int line_number)
{
	st_next();
}

*/