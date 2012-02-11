#ifndef ARCH_MISC_H
#define ARCH_MISC_H

#include "avr/pgmspace.h"

#define FLASH_STORED 	PSTR

void dev_print_flash(const char *s);
void dev_enable_ints();
void dev_disable_ints();
void delay_ms(uint16_t ms);
void delay_us(double time_us);

#endif
