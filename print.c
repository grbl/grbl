#include <math.h>
#include <avr/pgmspace.h>
#include "serial.h"

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

void printFloat(double n)
{
  double integer_part, fractional_part;
  fractional_part = modf(n, &integer_part);
  printInteger(integer_part);
  serial_write('.');
  printInteger(round(fractional_part*1000));
}

