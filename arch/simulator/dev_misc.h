#ifndef ARCH_MISC_H
#define ARCH_MISC_H

#define PSTR

void dev_print_flash(const char *s);
void dev_enable_ints();
void dev_disable_ints();
void delay_ms(unsigned int time_ms);
void delay_us(unsigned int time_us);
void sleep_mode();

#endif
