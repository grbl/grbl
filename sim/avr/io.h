/*
  interrupt.h - replacement for the avr include of the same name to provide
  dummy register variables and macros

  Part of Grbl Simulator

  Copyright (c) 2012-2104 Jens Geisler, Adam Shelly

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


#ifndef io_h
#define io_h

#include <inttypes.h>

union hilo16 {
  uint16_t w;
  struct {
    uint8_t l;  //TODO: check that these are right order on x86.  Doesn't matter for current usage, but might someday
    uint8_t h;
  };
};

enum {
  SIM_A, SIM_B, SIM_C, SIM_D, SIM_E,
  SIM_F, SIM_G, SIM_H, SIM_J,  SIM_K, SIM_L,
  SIM_PORT_COUNT
};

#define SIM_N_TIMERS 3 //328p has 3, Mega has 6


// dummy register variables
typedef struct io_sim {
  uint8_t ddr[SIM_PORT_COUNT]; 
  uint8_t port[SIM_PORT_COUNT];
  uint8_t pin[SIM_PORT_COUNT];
  uint8_t timsk[SIM_N_TIMERS];
  uint16_t ocra[SIM_N_TIMERS];
  uint16_t ocrb[SIM_N_TIMERS];
  uint16_t ocrc[SIM_N_TIMERS];
  uint16_t tcnt[SIM_N_TIMERS]; //tcint0 is really only 8bit
  uint8_t tccra[SIM_N_TIMERS];
  uint8_t tccrb[SIM_N_TIMERS];
  uint8_t tifr[SIM_N_TIMERS];
  uint8_t  pcicr;
  uint8_t pcmsk[3];
  uint8_t ucsr0[3];
  uint8_t udr[3];
  uint8_t gpior[3];
  uint8_t mcusr;
  uint8_t wdtcsr;
  union hilo16 ubrr0;

  uint16_t prescaler; //continuously running
  uint8_t sreg;

} io_sim_t;
volatile extern io_sim_t io;




// dummy macros for interrupt related registers
#define PORTA io.port[SIM_A]
#define PORTB io.port[SIM_B]
#define PORTC io.port[SIM_C]
#define PORTD io.port[SIM_D]
#define PORTE io.port[SIM_E]
#define PORTF io.port[SIM_F]
#define PORTG io.port[SIM_G]
#define PORTH io.port[SIM_H]
#define PORTJ io.port[SIM_J]
#define PORTK io.port[SIM_K]
#define PORTL io.port[SIM_L]

#define DDRA io.ddr[SIM_A]
#define DDRB io.ddr[SIM_B]
#define DDRC io.ddr[SIM_C]
#define DDRD io.ddr[SIM_D]
#define DDRE io.ddr[SIM_E]
#define DDRF io.ddr[SIM_F]
#define DDRG io.ddr[SIM_G]
#define DDRH io.ddr[SIM_H]
#define DDRJ io.ddr[SIM_J]
#define DDRK io.ddr[SIM_K]
#define DDRL io.ddr[SIM_L]

#define PINA io.pin[SIM_A]
#define PINB io.pin[SIM_B]
#define PINC io.pin[SIM_C]
#define PIND io.pin[SIM_D]
#define PINE io.pin[SIM_E]
#define PINF io.pin[SIM_F]
#define PING io.pin[SIM_G]
#define PINH io.pin[SIM_H]
#define PINJ io.pin[SIM_J]
#define PINK io.pin[SIM_K]
#define PINL io.pin[SIM_L]


#define TIMSK0 io.timsk[0]
#define TIMSK1 io.timsk[1]
#define TIMSK2 io.timsk[2]
#define TIMSK3 io.timsk[3]
#define TIMSK4 io.timsk[4]
#define TIMSK5 io.timsk[5]


#define SIM_TOV  0
#define SIM_OCA 1
#define SIM_OCB 2
#define SIM_OCC 3
#define SIM_ICI 5
#define SIM_ROLL 7  //stealing reserved TIFR bit

#define OCIE0A SIM_OCA
#define OCIE0B SIM_OCB
#define TOIE0  SIM_TOV

#define ICIE1   SIM_ICI
#define OCIE1C  SIM_OCC
#define OCIE1B  SIM_OCB
#define OCIE1A  SIM_OCA
#define TOIE1   SIM_ICI

#define ICIE2   SIM_ICI
#define OCIE2C  SIM_OCC
#define OCIE2B  SIM_OCB
#define OCIE2A  SIM_OCA
#define TOIE2   SIM_TOV

#define OCR0A io.ocra[0]
#define OCR1A io.ocra[1]
#define OCR2A io.ocra[2]
  //There are more..


#define TCNT0  io.tcnt[0]
#define TCNT1  io.tcnt[1]
#define TCNT2  io.tcnt[2]

#define TCCR0A io.tccra[0]
#define TCCR0B io.tccrb[0]
#define TCCR1A io.tccra[1]
#define TCCR1B io.tccrb[1]
#define TCCR2A io.tccra[2]
#define TCCR2B io.tccrb[2]

#define CS00 0
#define CS01 1
#define CS12 2
#define CS11 1
#define CS10 0
#define CS21 1

#define WGM13 4
#define WGM12 3
#define WGM11 1
#define WGM10 0
#define WGM21 1

#define COM1A1 7
#define COM1A0 6
#define COM1B1 5
#define COM1B0 4
#define COM1C1 3
#define COM1C0 2


#define PCICR io.pcicr
#define PCIE0 0
#define PCIE1 1
#define PCIE2 2

//serial channel
#define UCSR0A io.ucsr0[SIM_A]
#define UCSR0B io.ucsr0[SIM_B]
#define UDR0   io.udr[0]
#define UDRIE0 0
#define RXCIE0 1
#define RXEN0  2
#define TXEN0  3
#define U2X0   4
#define UBRR0H io.ubrr0.h
#define UBRR0L io.ubrr0.l

#define PCMSK0 io.pcmsk[0]
#define PCMSK1 io.pcmsk[1]
#define PCMSK2 io.pcmsk[2]

//GPIO
#define GPIOR0 io.gpior[0]
#define GPIOR1 io.gpior[1]
#define GPIOR2 io.gpior[2]

//MCU Status
#define MCUSR io.mcusr

#define PORF 0
#define EXTRF 1
#define BORF 2
#define WDRF 3
#define JTRF 4

//Interrupt Status
#define SREG io.sreg



#endif
