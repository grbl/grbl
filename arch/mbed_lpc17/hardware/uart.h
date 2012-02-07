/* DMA-driven UART driver for LPC17xx (should also work on LPC23xx but untested).
 *
 * Rob Turner, August 2010.
 */

/*
 *	adjusted by chrisu de 2011
 *
 *
 */

#ifndef UART_H_
#define UART_H_

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "LPC17xx.h"

/* which Uart interface is used for printf and debug... */
#define STD_UART 0

void uart_init(int device_num, size_t txFifoLen, size_t rxFifoLen, bool useDMA);	
void uart_SetBaud(int device_num, uint32_t baud);
int uart_read(int device_num, char * buf, size_t len);
int uart_write(int device_num, const char * buf, size_t len);
int uart_writeUnbuffered(int device_num, const char * buf, size_t len);

extern int uart_printf(const char *format, ...);

char uart_getc();
void uart_putc(char c);
void uart_print_binary(unsigned long mask);  

#endif // UART_H_

