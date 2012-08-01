/*
  limits.c - code pertaining to limit-switches and performing the homing cycle
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
  
#include <util/delay.h>
#include <avr/io.h>
#include "stepper.h"
#include "limits.h"
#include "settings.h"
#include "nuts_bolts.h"
#include "config.h"
#include "motion_control.h"
#include "planner.h"

void limits_init() {
  // I/O lines as inputs done by SETUP_IO() in config.h called from main()
}

static void homing_cycle(bool x_axis, bool y_axis, bool z_axis, bool reverse_direction, uint32_t microseconds_per_pulse) {
  uint32_t step_delay = microseconds_per_pulse - settings.pulse_microseconds;
  uint8_t out_bits = DIRECTION_MASK;
  uint8_t limit_bits;

  if (x_axis) { out_bits |= _BV(X_STEP_BIT); }
  if (y_axis) { out_bits |= _BV(Y_STEP_BIT); }
  if (z_axis) { out_bits |= _BV(Z_STEP_BIT); }

  // Invert direction bits if this is a reverse homing_cycle
  if (reverse_direction) {
    out_bits ^= DIRECTION_MASK;
  }

  // Apply the global invert mask
  out_bits ^= settings.invert_mask_stepdir;

  // Set direction pins, can't use |= because we may have 1 -> 0 transitions,
  // e.g. when reverse_direction is true
  STEPPING_PORT = (STEPPING_PORT & ~DIRECTION_MASK) | (out_bits & DIRECTION_MASK);

  for(;;) {
    limit_bits = LIMIT_PIN;

    if (reverse_direction) {
      // Invert limit_bits if this is a reverse homing_cycle
      limit_bits ^= LIMIT_MASK;
    }

    // Apply the global invert mask
    limit_bits ^= settings.invert_mask_limit;

    if (x_axis && !(limit_bits & _BV(X_LIMIT_BIT))) {
      x_axis = false;
      out_bits ^= _BV(X_STEP_BIT);
    }
    if (y_axis && !(limit_bits & _BV(Y_LIMIT_BIT))) {
      y_axis = false;
      out_bits ^= _BV(Y_STEP_BIT);
    }
    if (z_axis && !(limit_bits & _BV(Z_LIMIT_BIT))) {
      z_axis = false;
      out_bits ^= _BV(Z_STEP_BIT);
    }

    // Check if we are done
    if(!(x_axis || y_axis || z_axis)) { return; }
    // Send stepping pulse, can't use |= because we may have 1 -> 0 transitions,
    // e.g. when the STEP lines are inverted
    STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | (out_bits & STEP_MASK);
    delay_us(settings.pulse_microseconds);
    STEPPING_PIN = STEP_MASK; // End pulse via toggle, saves one port access
    delay_us(step_delay);
  }
  return;
}

// Usually all axes have the same resolution and when that's not the case, X and
// Y have identical resolutions and Z has more -- we're looking for the slowest
// going one, i.e. the one with the least resolution, thus X makes a good
// candidate
#define FEEDRATE_TO_PERIOD_US(f) (1.0 / (f) / 60 * settings.steps_per_mm[X_AXIS] * 100000)

static void approach_limit_switch(bool x, bool y, bool z) {
  homing_cycle(x, y, z, false, FEEDRATE_TO_PERIOD_US(settings.default_seek_rate));
}

static void leave_limit_switch(bool x, bool y, bool z) {
  homing_cycle(x, y, z, true, FEEDRATE_TO_PERIOD_US(settings.default_feed_rate));
}

void limits_go_home() {
  plan_synchronize();
  approach_limit_switch(false, false, true); // First home the z axis
  approach_limit_switch(true, true, false);  // Then home the x and y axis
  // Now carefully leave the limit switches
  leave_limit_switch(true, true, true);
  // Conclude that this is machine zero
  sys.position[X_AXIS] = sys.position[Y_AXIS] = sys.position[Z_AXIS] = 0;
}
