#ifndef acceleration_h
#define acceleration_h

// Estimate the maximum speed at a given distance when you need to reach the given 
// target_velocity with max_accelleration.
double estimate_max_speed(double max_accelleration, double target_velocity, double distance);

// At what distance must we start accellerating/braking to reach target_speed from current_speed given the 
// specified constant accelleration.
double estimate_brake_distance(double current_speed, double target_speed, double acceleration);

#endif