/*
  acceleration.c - support methods for acceleration-related calcul
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


// Estimate the maximum speed at a given distance when you need to reach the given 
// target_velocity with max_accelleration.
float estimate_max_speed(float max_accelleration, float target_velocity, float distance) {
  return(sqrt(-2*max_accelleration*distance+target_velocity*target_velocity))
}

// At what distance must we start accellerating/braking to reach target_speed from current_speed given the 
// specified constant accelleration.
float estimate_brake_distance(float current_speed, float target_speed, float acceleration) {
  return((target_speed*target_speed-current_speed*current_speed)/(2*acceleration));
}

// Calculate feed rate in length-units/second for a single axis
float axis_feed_rate(float steps_per_stepping, uint32_t stepping_rate, float steps_per_unit) {
  if (stepping_rate == 0) { return(0.0); }
  return((TICKS_PER_MICROSECOND*1000000)*steps_per_stepping/(stepping_rate*steps_per_unit));
}

// The 'swerve' of a joint is equal to the maximum accelleration of any single
// single axis in the corner between the outgoing and the incoming line. Accelleration control
// will regulate speed to avoid excessive swerve.
float calculate_swerve(struct Line* outgoing, struct Line* incoming) {  
  float x_swerve = abs(
    axis_feed_rate(
      ((float)incoming->steps_x)/incoming->maximum_steps, incoming->rate, settings.steps_per_mm[X_AXIS])
    - axis_feed_rate(
      ((float)incoming->steps_x)/incoming->maximum_steps, outgoing-> rate, settings.steps_per_mm[X_AXIS]));
  float y_swerve = abs(
    axis_feed_rate(
      ((float)incoming->steps_y)/incoming->maximum_steps, incoming->rate, settings.steps_per_mm[Y_AXIS])
    - axis_feed_rate(
      ((float)incoming->steps_y)/incoming->maximum_steps, outgoing-> rate, settings.steps_per_mm[Y_AXIS]));
  float z_swerve = abs(
    axis_feed_rate(
      ((float)incoming->steps_z)/incoming->maximum_steps, incoming->rate, settings.steps_per_mm[Z_AXIS])
    - axis_feed_rate(
      ((float)incoming->steps_z)/incoming->maximum_steps, outgoing-> rate, settings.steps_per_mm[Z_AXIS]));
  return max(x_swerve, max(y_swerve, z_swerve));
}

