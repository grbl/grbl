// Prepare an arc. theta == start angle, angular_travel == number of radians to go along the arc,
// positive angular_travel means clockwise, negative means counterclockwise. Radius == the radius of the
// circle in millimeters. axis_1 and axis_2 selects the plane in tool space. 
// ISSUE: The arc interpolator assumes all axes have the same steps/mm as the X axis.
void mc_arc(double theta, double angular_travel, double radius, int axis_1, int axis_2, double feed_rate)
{  
  uint32_t radius_steps = round(radius*X_STEPS_PER_MM);
  mc.mode = MC_MODE_ARC;
  // Determine angular direction (+1 = clockwise, -1 = counterclockwise)
  mc.arc.angular_direction = signof(angular_travel);
  // Calculate the initial position and target position in the local coordinate system of the arc
  mc.arc.x = round(sin(theta)*radius_steps); 
  mc.arc.y = round(cos(theta)*radius_steps);
  mc.arc.target_x = trunc(sin(theta+angular_travel)*radius_steps);
  mc.arc.target_y = trunc(cos(theta+angular_travel)*radius_steps);
  // Precalculate these values to optimize target detection
  mc.arc.target_direction_x = signof(mc.arc.target_x)*mc.arc.angular_direction;
  mc.arc.target_direction_y = signof(mc.arc.target_y)*mc.arc.angular_direction;
  // The "error" factor is kept up to date so that it is always == (x**2+y**2-radius**2). When error 
  // <0 we are inside the arc, when it is >0 we are outside of the arc, and when it is 0 we 
  // are exactly on top of the arc.
  mc.arc.error = mc.arc.x*mc.arc.x + mc.arc.y*mc.arc.y - radius_steps*radius_steps;
  // Because the error-value moves in steps of (+/-)2x+1 and (+/-)2y+1 we save a couple of multiplications
  // by keeping track of the doubles of the arc coordinates at all times.
  mc.arc.x2 = 2*mc.arc.x;
  mc.arc.y2 = 2*mc.arc.y; 
  
  // Set up a vector with the steppers we are going to use tracing the plane of this arc
  clear_vector(mc.arc.plane_steppers);
  mc.arc.plane_steppers[axis_1] = 1;
  mc.arc.plane_steppers[axis_2] = 1;
  // And map the local coordinate system of the arc onto the tool axes of the selected plane
  mc.arc.axis_x = axis_1;
  mc.arc.axis_y = axis_2;
  // mm/second -> microseconds/step. Assumes all axes have the same steps/mm as the x axis
  mc.pace = 
    ONE_MINUTE_OF_MICROSECONDS / (feed_rate * X_STEPS_PER_MM);
  mc.arc.incomplete = true;
}

#define check_arc_target \
  if ((mc.arc.x * mc.arc.target_direction_y >= \
          mc.arc.target_x * mc.arc.target_direction_y) && \
         (mc.arc.y * mc.arc.target_direction_x <= \
          mc.arc.target_y * mc.arc.target_direction_x)) \
  { mc.arc.incomplete = false; }

// Internal method used by execute_arc to trace horizontally in the general direction provided by dx and dy
void step_arc_along_x(int8_t dx, int8_t dy) 
{
  uint32_t diagonal_error;
  mc.arc.x+=dx;
  mc.arc.error += 1+mc.arc.x2*dx;
  mc.arc.x2 += 2*dx;
  diagonal_error = mc.arc.error + 1 + mc.arc.y2*dy;
  if(abs(mc.arc.error) >= abs(diagonal_error)) {
    mc.arc.y += dy;
    mc.arc.y2 += 2*dy;
    mc.arc.error = diagonal_error;
    step_steppers(mc.arc.plane_steppers); // step diagonal
  } else {
    step_axis(mc.arc.axis_x); // step straight
  }
  check_arc_target;
}

// Internal method used by execute_arc to trace vertically in the general direction provided by dx and dy
void step_arc_along_y(int8_t dx, int8_t dy) 
{  
  uint32_t diagonal_error;
  mc.arc.y+=dy; 
  mc.arc.error += 1+mc.arc.y2*dy; 
  mc.arc.y2 += 2*dy; 
  diagonal_error = mc.arc.error + 1 + mc.arc.x2*dx; 
  if(abs(mc.arc.error) >= abs(diagonal_error)) { 
    mc.arc.x += dx; 
    mc.arc.x2 += 2*dx; 
    mc.arc.error = diagonal_error; 
    step_steppers(mc.arc.plane_steppers); // step diagonal
  } else {
    step_axis(mc.arc.axis_y); // step straight
  }
  check_arc_target;
}

// Take dx and dy which are local to the arc being generated and map them on to the 
// selected tool-space-axes for the current arc.
void map_local_arc_directions_to_stepper_directions(int8_t dx, int8_t dy)
{
  int8_t direction[3];
  direction[mc.arc.axis_x] = dx;
  direction[mc.arc.axis_y] = dy;
  set_stepper_directions(direction);
}



/*
 Quandrants of the arc
       \ 7|0 /
        \ | / 
      6  \|/  1    y+
 ---------|-----------
      5  /|\  2    y-
        / | \  
   x-  / 4|3 \ x+           */

#ifdef UNROLLED_ARC_LOOP // This function only used by the unrolled arc loop
// Determine within which quadrant of the circle the provided coordinate falls
int quadrant(uint32_t x,uint32_t y)
{
  // determine if the coordinate is in the quadrants 0,3,4 or 7
  register int quad0347 = abs(x)<abs(y);
  
  if (x<0) { // quad 4567
    if (y<0) { // quad 45
      return(quad0347 ? 4 : 5);
    } else { // quad 67
      return(quad0347 ? 7 : 6);
    }
  } else {
    if (y<0) { // quad 23
      return(quad0347 ? 3 : 2);
    } else { // quad 01
      return(quad0347 ? 0 : 1);
    }
  }  
}
#endif

// Will trace the configured arc until the target is reached. 
void execute_arc()
{
  uint32_t start_x = mc.arc.x;
  uint32_t start_y = mc.arc.y;
  int dx, dy; // Trace directions
  int steps = 0;

  // mc.mode is set to 0 (MC_MODE_AT_REST) when target is reached
  while(mc.arc.incomplete && (steps<400))
  {
    steps++;
#ifdef UNROLLED_ARC_LOOP
    // Unrolling the arc code is fast, but costs about 830 bytes of extra code space.
    int q = quadrant(mc.arc.x, mc.arc.y);
    if (mc.arc.angular_direction) {
      switch (q) {
        case 0: 
        map_local_arc_directions_to_stepper_directions(1,-1);
        while(mc.arc.incomplete && (mc.arc.x>mc.arc.y)) { step_arc_along_x(1,-1); }
        case 1: 
        map_local_arc_directions_to_stepper_directions(1,-1);
        while(mc.arc.incomplete && (mc.arc.y>0)) { step_arc_along_y(1,-1); }
        case 2: 
        map_local_arc_directions_to_stepper_directions(-1,-1);
        while(mc.arc.incomplete && (mc.arc.y>-mc.arc.x)) { step_arc_along_y(-1,-1); }
        case 3: 
        map_local_arc_directions_to_stepper_directions(-1,-1);
        while(mc.arc.incomplete && (mc.arc.x>0)) { step_arc_along_x(-1,-1); }
        case 4: 
        map_local_arc_directions_to_stepper_directions(-1,1);
        while(mc.arc.incomplete && (mc.arc.y<mc.arc.x)) { step_arc_along_x(-1,1); }
        case 5: 
        map_local_arc_directions_to_stepper_directions(-1,1);
        while(mc.arc.incomplete && (mc.arc.y<0)) { step_arc_along_y(-1,1); }
        case 6: 
        map_local_arc_directions_to_stepper_directions(1,-1);
        while(mc.arc.incomplete && (mc.arc.y<-mc.arc.x)) { step_arc_along_y(1,1); }
        case 7: 
        map_local_arc_directions_to_stepper_directions(1,1);
        while(mc.arc.incomplete && (mc.arc.x<0)) { step_arc_along_x(1,1); }
      }
    } else {
      switch (q) {
        case 7:
        map_local_arc_directions_to_stepper_directions(-1,-1);
        while(mc.arc.incomplete && (mc.arc.y>-mc.arc.x)) { step_arc_along_x(-1,-1); }
        case 6:
        map_local_arc_directions_to_stepper_directions(-1,-1);
        while(mc.arc.incomplete && (mc.arc.y>0))  { step_arc_along_y(-1,-1); }
        case 5:
        map_local_arc_directions_to_stepper_directions(1,-1);
        while(mc.arc.incomplete && (mc.arc.y>mc.arc.x))  { step_arc_along_y(1,-1); }
        case 4:
        map_local_arc_directions_to_stepper_directions(1,-1);
        while(mc.arc.incomplete && (mc.arc.x<0))  { step_arc_along_x(1,-1); }
        case 3:
        map_local_arc_directions_to_stepper_directions(1,1);
        while(mc.arc.incomplete && (mc.arc.y<-mc.arc.x)) { step_arc_along_x(1,1); }      
        case 2:
        map_local_arc_directions_to_stepper_directions(1,1);
        while(mc.arc.incomplete && (mc.arc.y<0))  { step_arc_along_y(1,1); }      
        case 1:
        map_local_arc_directions_to_stepper_directions(-1,1);
        while(mc.arc.incomplete && (mc.arc.y<mc.arc.x))  { step_arc_along_y(-1,1); }
        case 0:
        map_local_arc_directions_to_stepper_directions(-1,1);
        while(mc.arc.incomplete && (mc.arc.x>0))  { step_arc_along_x(-1,1); }
      }    
    }
#else 
    dx = (mc.arc.y!=0) ?  signof(mc.arc.y) * mc.arc.angular_direction : -signof(mc.arc.x);
    dy = (mc.arc.x!=0) ? -signof(mc.arc.x) * mc.arc.angular_direction : -signof(mc.arc.y);
    map_local_arc_directions_to_stepper_directions(dx,dy);
    if (abs(mc.arc.x)<abs(mc.arc.y)) {
      step_arc_along_x(dx,dy);    
    } else {
      
      step_arc_along_y(dx,dy);    
    }
#endif    
  }
  // Update the tool position to the new actual position
  mc.position[mc.arc.axis_x] += mc.arc.x-start_x;
  mc.position[mc.arc.axis_y] += mc.arc.y-start_y;
  // Because of rounding errors we might be off by a step or two. Adjust for this
    // To be implemented
  //void prepare_linear_motion(uint32_t x, uint32_t y, uint32_t z, float feed_rate, int invert_feed_rate)
  mc.mode = MC_MODE_AT_REST;  
}
