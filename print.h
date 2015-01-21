/*
  print.h - Functions for formatting output strings
  Part of Horus Firmware

  Copyright (c) 2014-2015 Mundo Reader S.L.

  Horus Firmware is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Horus Firmware is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Horus Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/
/* 
  This file is based on work from Grbl v0.9, distributed under the 
  terms of the GPLv3. See COPYING for more details.
    Copyright (c) 2009-2011 Simen Svale Skogsrud
    Copyright (c) 2011-2014 Sungeun K. Jeon
*/ 

#ifndef print_h
#define print_h


void printString(const char *s);

void printPgmString(const char *s);

void printInteger(long n);

void print_uint32_base10(uint32_t n);

void print_uint8_base2(uint8_t n);

void print_uint8_base10(uint8_t n);

void printFloat(float n, uint8_t decimal_places);

// Floating value printing handlers for special variables types used in Grbl. 
//  - CoordValue: Handles all position or coordinate values in inches or mm reporting.
//  - RateValue: Handles feed rate and current velocity in inches or mm reporting.
//  - SettingValue: Handles all floating point settings values (always in mm.)
void printFloat_CoordValue(float n);

void printFloat_RateValue(float n);

void printFloat_SettingValue(float n);

// Debug tool to print free memory in bytes at the called point. Not used otherwise.
void printFreeMemory();

#endif