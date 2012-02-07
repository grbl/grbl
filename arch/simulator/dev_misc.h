#ifndef ARCH_MISC_H
#define ARCH_MISC_H

#define FLASH_STORED 	

void dev_print_flash(const char *s);
void dev_enable_ints();
void dev_disable_ints();
void delay_ms(double time_ms);
void delay_us(double time_us);

#endif
