#include "serial.h"
#include "dev_misc.h"
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void dev_print_flash(const char *s)
{
	char c;
	while ((c = pgm_read_byte_near(s++)))
		serial_write(c);
}

void dev_enable_ints()
{
	sei();
}

void dev_disable_ints()
{
	cli();
}

// Delays variable defined milliseconds. Compiler compatibility fix for _delay_ms(),
// which only accepts constants in future compiler releases.
void delay_ms(uint16_t ms) 
{
  while ( ms-- ) { _delay_ms(1); }
}

void delay_us(double time_us)
{
	_delay_us(time_us);
}

extern int grbl_main();
int main() 
{
	grbl_main();	
}
