#include "serial.h"
#include "dev_misc.h"
#include <stdio.h>



void dev_print_flash(const char *s) // FLASH_STORED needs to be defined
{
	printf(s);
}

void dev_enable_ints()
{
//	sei();
}

void dev_disable_ints()
{
//	cli();
}

void delay_ms(double time_ms) 
{
//	_delay_ms(time_ms);
}

void delay_us(double time_us)
{
//	_delay_us(time_us);
}

extern int grbl_main();
int main() 
{
	printf("starting\n");
	grbl_main();	
}
