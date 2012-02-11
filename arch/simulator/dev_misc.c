#include "serial.h"
#include "dev_misc.h"
#include <stdio.h>
#include <unistd.h>

int exit_app = 0;

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

void delay_ms(unsigned int time_ms) 
{
	usleep(time_ms * 1000);
}

void delay_us(unsigned int time_us)
{
	usleep(time_us);
}

void sleep_mode()
{

}

extern int grbl_main();
int main() 
{
	printf("starting\n");
	grbl_main();
}

