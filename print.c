/*
  print.c - Functions for formatting output strings
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

/* This code was initially inspired by the wiring_serial module by David A. Mellis which
   used to be a part of the Arduino project. */ 


#include <math.h>
#include <avr/pgmspace.h>
#include "serial.h"

#ifndef DECIMAL_PLACES
#define DECIMAL_PLACES 3
#define DECIMAL_MULTIPLIER 1000
#endif

void printString(const char *s)
{
  while (*s)
    serial_write(*s++);
}

// Print a string stored in PGM-memory
void printPgmString(const char *s)
{
  char c;
  while ((c = pgm_read_byte_near(s++)))
    serial_write(c);
}

void printIntegerInBase(unsigned long n, unsigned long base)
{ 
	unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars. 
	unsigned long i = 0;

	if (n == 0) {
		serial_write('0');
		return;
	} 

	while (n > 0) {
		buf[i++] = n % base;
		n /= base;
	}

	for (; i > 0; i--)
		serial_write(buf[i - 1] < 10 ?
			'0' + buf[i - 1] :
			'A' + buf[i - 1] - 10);
}

void printInteger(long n)
{
	if (n < 0) {
		serial_write('-');
		n = -n;
	}

	printIntegerInBase(n, 10);
}

// A very simple 
void printFloat(double n)
{
  if (n < 0) {
    serial_write('-');
    n = -n;
  }
  n += 0.5/DECIMAL_MULTIPLIER; // Add rounding factor
 
  long integer_part;
  integer_part = (int)n;
  printIntegerInBase(integer_part,10);
  
  serial_write('.');
  
  n -= integer_part;
  int decimals = DECIMAL_PLACES;  
  uint8_t decimal_part;  
  while(decimals-- > 0) {
    n *= 10;
    decimal_part = (int) n;
    serial_write('0'+decimal_part);
    n -= decimal_part;
  }
}
