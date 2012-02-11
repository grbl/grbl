/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
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

#include "config.h"
#include "serial.h"
#include "print.h"
#include "nuts_bolts.h"

#include <stm32f10x.h>
#include <stm32f10x_conf.h>

#define RX_BUFFER_SIZE 128
#define TX_BUFFER_SIZE 64

static char tx_buffer[TX_BUFFER_SIZE];
static char rx_buffer[RX_BUFFER_SIZE];

static uint8_t tx_buffer_head = 0;
static uint8_t tx_buffer_tail = 0;

static uint8_t rx_buffer_head = 0;
static uint8_t rx_buffer_tail = 0;

void dev_print_flash(const char *s) 
{
	printString(s);
}

void serial_init(long baud)
{
	USART_InitTypeDef usart_init;

	usart_init.USART_BaudRate = baud;
	usart_init.USART_WordLength = USART_WordLength_8b;
	usart_init.USART_StopBits = 1;
	usart_init.USART_Parity = USART_Parity_No;
	usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &usart_init);

	/* Enable the USART */
	USART_Cmd(USART1, ENABLE);
}

void serial_write(uint8_t data)
{
	uint8_t next_head = tx_buffer_head + 1;
	if(next_head >= TX_BUFFER_SIZE) {
		next_head = 0;
	}
  	// Wait until there is space in the buffer
	while (next_head == tx_buffer_tail);	
	tx_buffer[tx_buffer_head] = data;
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

	tx_buffer_head = next_head;
	return;
}

uint8_t serial_read()
{
	if(rx_buffer_head == rx_buffer_tail) {
		return SERIAL_NO_DATA;
	}  
	uint8_t c = rx_buffer[rx_buffer_tail];
	rx_buffer_tail ++;
	if(rx_buffer_tail >= RX_BUFFER_SIZE) {
		rx_buffer_tail = 0;
	}
	return c;
}

void serial_reset_read_buffer() 
{
	rx_buffer_tail = rx_buffer_head;
}

/**
 * @brief  This function handles USART interrupt request.
 * @param  None
 * @retval None
 */
void usart1_isr(void)
{
	unsigned char ch;

	if (USART_GetITStatus(USART1, USART_IT_TXE) != 0x00) {
		if (tx_buffer_tail != tx_buffer_head) {
			ch = tx_buffer[tx_buffer_tail];

			USART_SendData(USART1, ch);
			tx_buffer_tail ++;
			if(tx_buffer_tail >= TX_BUFFER_SIZE) {
				tx_buffer_tail = 0;
			}
		}
		else{
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
		}
	}
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != 0x00) {
		rx_buffer[rx_buffer_head] = USART_ReceiveData(USART1);
		//USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);  //--> should not be needed
		rx_buffer_head ++;
		if(rx_buffer_head >= RX_BUFFER_SIZE) {
			rx_buffer_head = 0;
		}
	}
}

/* TODO    
// Pick off runtime command characters directly from the serial stream. These characters are
    // not passed into the buffer, but these set system state flag bits for runtime execution.
    // 		switch (data) {
		case CMD_STATUS_REPORT: sys_state |= BIT_STATUS_REPORT; break; // Set as true
		case CMD_CYCLE_START:   sys_state |= BIT_CYCLE_START; break; // Set as true
		case CMD_FEED_HOLD:     sys_state |= BIT_FEED_HOLD; break; // Set as true
		case CMD_RESET: 
       		// Immediately force stepper subsystem idle at an interrupt level.
		if (!(sys_state & BIT_RESET)) { // Force stop only first time.
			st_go_idle();  
		}
       		sys_state |= BIT_RESET; // Set as true
       		break;

         default : // Write character to buffer
        rx_buffer[rx_buffer_head] = data;
        rx_buffer_head = next_head;
*/

