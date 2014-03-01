/*
  report.h - reporting and messaging methods
  Part of Grbl

  Copyright (c) 2012-2014 Sungeun K. Jeon

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
#define STATUS_ARC_RADIUS_ERROR 4
#define STATUS_MODAL_GROUP_VIOLATION 5
#define STATUS_INVALID_STATEMENT 6
#define STATUS_SETTING_DISABLED 7
#define STATUS_SETTING_VALUE_NEG 8
#define STATUS_SETTING_STEP_PULSE_MIN 9
#define STATUS_SETTING_READ_FAIL 10
#define STATUS_IDLE_ERROR 11
#define STATUS_ALARM_LOCK 12
#define STATUS_SOFT_LIMIT_ERROR 13
#define STATUS_OVERFLOW 14

// Define Grbl alarm codes. Less than zero to distinguish alarm error from status error.
#define ALARM_LIMIT_ERROR -1
#define ALARM_ABORT_CYCLE -2
#define ALARM_PROBE_FAIL -3

// Define Grbl feedback message codes.
#define MESSAGE_CRITICAL_EVENT 1
#define MESSAGE_ALARM_LOCK 2
#define MESSAGE_ALARM_UNLOCK 3
#define MESSAGE_ENABLED 4
#define MESSAGE_DISABLED 5

// Prints system status messages.
void report_status_message(uint8_t status_code);

// Prints system alarm messages.
void report_alarm_message(int8_t alarm_code);

// Prints miscellaneous feedback messages.
void report_feedback_message(uint8_t message_code);

// Prints welcome message
void report_init_message();

// Prints Grbl help and current global settings
void report_grbl_help();

// Prints Grbl global settings
void report_grbl_settings();

// Prints realtime status report
void report_realtime_status();

// Prints recorded probe position
void report_probe_parameters();

// Prints Grbl NGC parameters (coordinate offsets, probe)
void report_ngc_parameters();

// Prints current g-code parser mode state
void report_gcode_modes();

// Prints startup line
void report_startup_line(uint8_t n, char *line);

// Prints build info and user info
void report_build_info(char *line);

#endif
