#include "spi.h"

void spi_attachInterrupt() {
  SPCR |= _BV(SPIE);
}

void spi_detachInterrupt() {
  SPCR &= ~_BV(SPIE);
}

uint8_t spi_transfer(uint8_t _data) {
  SPDR = _data;
  while (!(SPSR & _BV(SPIF)))
    ;
  return SPDR;
}

void spi_begin() {
  // Set SS to high so a connected chip will be "deselected" by default
  // digitalWrite(SS, HIGH);
  PORTB |= _BV(PIN2);

  // When the SS pin is set as OUTPUT, it can be used as
  // a general purpose output port (it doesn't influence
  // SPI operations).
  // pinMode(SS, OUTPUT);
  DDRB |= _BV(DD2);

  // Warning: if the SS pin ever becomes a LOW INPUT then SPI
  // automatically switches to Slave, so the data direction of
  // the SS pin MUST be kept as OUTPUT.
  SPCR |= _BV(MSTR);
  SPCR |= _BV(SPE);

  // Set direction register for SCK and MOSI pin.
  // MISO pin automatically overrides to INPUT.
  // By doing this AFTER enabling SPI, we avoid accidentally
  // clocking in a single bit since the lines go directly
  // from "input" to SPI control.  
  // http://code.google.com/p/arduino/issues/detail?id=888
  //pinMode(SCK, OUTPUT);
  //pinMode(MOSI, OUTPUT);
  
  DDRB |= _BV(DD5);
  DDRB |= _BV(DD3);
}


void spi_end() {
  SPCR &= ~_BV(SPE);
}

void spi_setBitOrder(uint8_t bitOrder) {
  if(bitOrder == LSBFIRST) {
    SPCR |= _BV(DORD);
  } else {
    SPCR &= ~(_BV(DORD));
  }
}

void spi_setDataMode(uint8_t mode) {
  SPCR = (SPCR & ~SPI_MODE_MASK) | mode;
}

void spi_setClockDivider(uint8_t rate)
{
  SPCR = (SPCR & ~SPI_CLOCK_MASK) | (rate & SPI_CLOCK_MASK);
  SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | ((rate >> 2) & SPI_2XCLOCK_MASK);
}


