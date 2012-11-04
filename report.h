/*
  report.h - reporting and messaging methods
  Part of Grbl

  Copyright (c) 2012 Sungeun K. Jeon

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
#ifndef report_h
#define report_h


// Define Grbl status codes.
#define STATUS_OK 0
#define STATUS_BAD_NUMBER_FORMAT 1
#define STATUS_EXPECTED_COMMAND_LETTER 2
#define STATUS_UNSUPPORTED_STATEMENT 3
#define STATUS_FLOATING_POINT_ERROR 4
#define STATUS_MODAL_GROUP_VIOLATION 5
#define STATUS_INVALID_STATEMENT 6
#define STATUS_HARD_LIMIT 7
#define STATUS_SETTING_DISABLED 8
#define STATUS_SETTING_VALUE_NEG 9
#define STATUS_SETTING_STEP_PULSE_MIN 10
#define STATUS_SETTING_READ_FAIL 11
#define STATUS_HOMING_ERROR 12
#define STATUS_ABORT_CYCLE 13
#define STATUS_PURGE_CYCLE 14

// Define Grbl feedback message codes. Less than zero to distinguish message from error.
#define MESSAGE_SYSTEM_ALARM -1
#define MESSAGE_POSITION_LOST -2
#define MESSAGE_HOMING_ENABLE -3
#define MESSAGE_SWITCH_ON -4
#define MESSAGE_SWITCH_OFF -5
#define MESSAGE_PURGE_AXES_LOCK -6

// Prints system status messages.
void report_status_message(uint8_t status_code);

// Prints miscellaneous feedback messages.
void report_feedback_message(int8_t message_code);

// Prints welcome message
void report_init_message();

// Prints Grbl help and current global settings
void report_grbl_help();

// Prints Grbl global settings
void report_grbl_settings();

// Prints realtime status report
void report_realtime_status();

// Prints Grbl persistent coordinate parameters
void report_gcode_parameters();

// Prints current g-code parser mode state and active switches
void report_gcode_modes();

// Prints startup line
void report_startup_line(uint8_t n, char *line);

#endif
