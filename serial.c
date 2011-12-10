/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "serial.h"


#ifdef __AVR_ATmega328P__
#define RX_BUFFER_SIZE 256
#else
#define RX_BUFFER_SIZE 64
#endif

#define TX_BUFFER_SIZE 16

uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_buffer_head = 0;
uint8_t rx_buffer_tail = 0;

uint8_t tx_buffer[TX_BUFFER_SIZE];
uint8_t tx_buffer_head = 0;
volatile uint8_t tx_buffer_tail = 0;

static void set_baud_rate(long baud) {
  uint16_t UBRR0_value = ((F_CPU / 16 + baud / 2) / baud - 1);
  UBRR0H = UBRR0_value >> 8;
  UBRR0L = UBRR0_value;
}

void serial_init(long baud)
{
  set_baud_rate(baud);
  
  /* baud doubler off  - Only needed on Uno XXX */
  UCSR0A &= ~(1 << U2X0);
          
  // enable rx and tx
  UCSR0B |= 1<<RXEN0;
  UCSR0B |= 1<<TXEN0;
	
  // enable interrupt on complete reception of a byte
  UCSR0B |= 1<<RXCIE0;
	  
  // defaults to 8-bit, no parity, 1 stop bit
}

void serial_write(uint8_t data) {
  // Calculate next head
  uint8_t next_head = tx_buffer_head + 1;
  if (next_head == TX_BUFFER_SIZE) { next_head = 0; }

  // Wait until there's a space in the buffer
  while (next_head == tx_buffer_tail) { };//sleep_mode(); };

  // Store data and advance head
  tx_buffer[tx_buffer_head] = data;
  tx_buffer_head = next_head;
  
  // Enable Data Register Empty Interrupt to make sure tx-streaming is running
  UCSR0B |=  (1 << UDRIE0); 
}

// Data Register Empty Interrupt handler
ISR(USART_UDRE_vect) {  
  // Temporary tx_buffer_tail (to optimize for volatile)
  uint8_t tail = tx_buffer_tail;

  // Send a byte from the buffer	
  UDR0 = tx_buffer[tail];

  // Update tail position
  tail ++;
  if (tail == TX_BUFFER_SIZE) { tail = 0; }

  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
  if (tail == tx_buffer_head) { UCSR0B &= ~(1 << UDRIE0); }

  tx_buffer_tail = tail;
}

uint8_t serial_read()
{
  if (rx_buffer_head == rx_buffer_tail) {
    return SERIAL_NO_DATA;
  } else {
    uint8_t data = rx_buffer[rx_buffer_tail];
    rx_buffer_tail++;
    if (rx_buffer_tail == RX_BUFFER_SIZE) { rx_buffer_tail = 0; }
    return data;
  }
}

ISR(USART_RX_vect)
{
  uint8_t data = UDR0;
  uint8_t next_head = rx_buffer_head + 1;
  if (next_head == RX_BUFFER_SIZE) { next_head = 0; }

  // Write data to buffer unless it is full.
  if (next_head != rx_buffer_tail) {
    rx_buffer[rx_buffer_head] = data;
    rx_buffer_head = next_head;
  }
}
