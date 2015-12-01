#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#define LSBFIRST 0
#define MSBFIRST 1

#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV16 0x01
#define SPI_CLOCK_DIV64 0x02
#define SPI_CLOCK_DIV128 0x03
#define SPI_CLOCK_DIV2 0x04
#define SPI_CLOCK_DIV8 0x05
#define SPI_CLOCK_DIV32 0x06
//#define SPI_CLOCK_DIV64 0x07

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

#define SPI_MODE_MASK 0x0C  // CPOL = bit 3, CPHA = bit 2 on SPCR
#define SPI_CLOCK_MASK 0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK 0x01  // SPI2X = bit 0 on SPSR

extern uint8_t spi_transfer(uint8_t);

extern void spi_attachInterrupt();
extern void spi_detachInterrupt();
extern void spi_begin();
extern void spi_end();
extern void spi_setBitOrder(uint8_t);
extern void spi_setDataMode(uint8_t);
extern void spi_setClockDivider(uint8_t);


#endif
