/*
  lcd.c - Support for 20x4 LCD using HD44780 in 4-bit mode via shift register 
  Part of Grbl fork to support stepduino

  Copyright (c) 2014 Freetronics Pty Ltd

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http:www.gnu.org/licenses/>.
*/

#include <stdint.h>
#include "system.h"
#include "lcd.h"
#include "report.h"
#include "settings.h"
#include "print.h"
#include "planner.h"
#include "spindle_control.h"
#include "gcode.h"

#define CMD_CLEAR_SCREEN 0x01
#define CMD_HOME 0x02
#define CMD_DEFAULT_ENTRYMODE 0x06 /* Increment address, don't scroll display */
#define CMD_MODE_HIDE_CURSOR 0x0C
#define CMD_SET_ADDR 0x80
#define CMD_BLANK 0x08

/* Write the specified byte to the shift register */
static inline void clock_data(uint8_t data)
{
  uint8_t bit;
  for(bit = 0; bit < 8; bit++) {
    if(data & _BV(bit))
      LCD_DATA_PORT |= _BV(LCD_DATA_BIT);
    else
      LCD_DATA_PORT &= ~_BV(LCD_DATA_BIT);
    LCD_CLOCK_PORT |= _BV(LCD_CLOCK_BIT);
    LCD_CLOCK_PORT &= ~_BV(LCD_CLOCK_BIT);
  }
  LCD_STROBE_PORT |= _BV(LCD_STROBE_BIT);
  LCD_STROBE_PORT &= ~_BV(LCD_STROBE_BIT);
}

/* Write a generic byte (command or memory write) to the display as 2
   4-bt writes, applying the specified flags and pausing for the specified
   delay.
*/
static inline void lcd_byte(uint8_t data, uint8_t flags, uint8_t delay_time)
{
  uint8_t hi = (data >> 4) | flags;
  uint8_t lo = (data & 0x0F) | flags;
  clock_data(hi | _BV(SHIFT_E));
  clock_data(hi);
  delay_us(delay_time);
  clock_data(lo | _BV(SHIFT_E));
  clock_data(lo);
  delay_us(delay_time);
}

/* Write a nibble to the display, used during reset sequence */
static inline void lcd_nibble(uint8_t nibble)
{
  clock_data(nibble | _BV(SHIFT_E));
  clock_data(nibble);
}

static void lcd_command(uint8_t cmd)
{
  lcd_byte(cmd, 0, 37);
}

static void lcd_write_dram(uint8_t data)
{
  lcd_byte(data, _BV(SHIFT_RS), 50);
}

static inline uint8_t address_of(uint8_t row, uint8_t col)
{
  switch(row) {
  case 0:
    return col;
  case 1:
    return 0x40+col;
  case 2:
    return 0x14+col;
  case 3:
    return 0x54+col;
  }
  return 0;
}

static void lcd_printPgmString(const char *s, uint8_t col, uint8_t row);
static void lcd_printString(char *s, uint8_t col, uint8_t row);
static void lcd_printChar(char c, uint8_t col, uint8_t row);
static void lcd_print_integer(long n, uint8_t col, uint8_t row, uint8_t padlen);

void lcd_init(void)
{
  // Strobe output, idles low
  LCD_STROBE_PORT &= ~_BV(LCD_STROBE_BIT);
  LCD_STROBE_DDR |= _BV(LCD_STROBE_BIT);
  // Clock output, idles low
  LCD_CLOCK_PORT &= ~_BV(LCD_CLOCK_BIT);
  LCD_CLOCK_DDR |= _BV(LCD_CLOCK_BIT);
  // Data output
  LCD_DATA_DDR |= _BV(LCD_DATA_BIT);

  /* Reset/init sequence taken from http://elm-chan.org/docs/lcd/hd44780_e.html */
  delay_ms(20);

  lcd_nibble(0x03); /* enter 8-bit mode */
  delay_ms(5);

  lcd_nibble(0x03);
  delay_ms(1);

  lcd_nibble(0x03);
  delay_ms(1);

  lcd_nibble(0x02); /* enter 4-bit mode */
  delay_ms(1);

  lcd_command(0x28); /* enter 4-bit 2-row mode */
  delay_ms(1);

  lcd_command(CMD_MODE_HIDE_CURSOR);
  lcd_command(CMD_CLEAR_SCREEN);
  delay_ms(2);
  lcd_command(CMD_DEFAULT_ENTRYMODE);

  lcd_printPgmString(PSTR("WELCOME TO GRBL"), 2, 1);
  lcd_printPgmString(PSTR("for StepDuino"), 3, 2);
  delay_ms(1000);
  lcd_clearScreen();

  lcd_printPgmString(PSTR("X:"), 0, 0);
  lcd_printPgmString(PSTR("Y:"), 0, 1);
  lcd_printPgmString(PSTR("F:"), 10, 0);
  lcd_printPgmString(PSTR("Sp:"), 10, 1);
  #ifdef USE_LINE_NUMBERS
  lcd_printPgmString(PSTR("Ln:"), 8, 2);
  #endif
}

void lcd_updateAxes()
{
  /* This function is called at a high frequency, so make sure not
     to do anything too slow here. Caching last responses is worthwhile. */

  /* first and second rows, axis information */
  int32_t current_position[N_AXIS];
  memcpy(current_position,sys.position,sizeof(sys.position));

  int i;
  for(i = 0; i < N_AXIS; i++) {
    current_position[i] /= settings.steps_per_mm[i];
    lcd_print_integer(current_position[i], 3, i, 5);
  }

  /* third row (index 2), machine information */

  /* machine global state */
  static const char *last;
  const char *state = machine_state_to_pstr();
  if(state != last) {
    lcd_printPgmString(state, 0, 2);
    last = state;
  }
}

void lcd_latestCommand(char *cmd)
{
  /* first row (index 0) */
  lcd_print_integer(gc_state.feed_rate, 12, 0, 6);
  lcd_printChar(settings.steps_per_mm[X_AXIS] < gc_state.feed_rate ? '!' : ' ', 18, 0);
  lcd_printChar(settings.steps_per_mm[Y_AXIS] < gc_state.feed_rate ? '!' : ' ', 19, 0);
  if(gc_state.modal.spindle != SPINDLE_DISABLE)
    lcd_print_integer(gc_state.spindle_speed, 13, 1, 7);
  else
    lcd_printPgmString(PSTR(" (Off) "), 13, 1);

  /* second row (index 1) */

  /* third row (index 2) */
  #ifdef USE_LINE_NUMBERS
  plan_block_t * pb = plan_get_current_block();
  if(pb != NULL) {
    int32_t ln;
    ln = pb->line_number;
    lcd_print_integer(ln, 10, 2, 10);
  }
#endif

  /* all on bottom row (index 3) */

  /* Spinning "rotor" as commands go past */
  static uint8_t last = 0;
  static const uint8_t rotor[] PROGMEM = { '|','-' };

  /* Current command modal group */
  lcd_printPgmString(gcode_mode_to_pstr(), 2, 3);

  /* Last received command */
  lcd_printChar(pgm_read_byte_near(rotor + last), 0, 3);
  last = (last + 1) % sizeof(rotor);
  lcd_printString(cmd,8,3);
  uint8_t i;
  for(i = strlen(cmd)+8; i < 20; i++)
    lcd_printChar(' ',i,3);
}

static void lcd_printString(char *s, uint8_t col, uint8_t row)
{
  lcd_command(CMD_SET_ADDR + address_of(row, 0) + col);
  while (*s) {
    lcd_write_dram(*s++);
    if(++col == 20)
      break;
  }
}

static void lcd_printPgmString(const char *s, uint8_t col, uint8_t row)
{
  lcd_command(CMD_SET_ADDR + address_of(row, 0) + col);
  char c;
  while ((c = pgm_read_byte_near(s++))) {
    lcd_write_dram(c);
    if(++col == 20)
      break;
  }
}

static void lcd_printChar(char c, uint8_t col, uint8_t row)
{
  lcd_command(CMD_SET_ADDR + address_of(row, 0) + col);
  lcd_write_dram(c);
}

static void lcd_print_integer(long n, uint8_t col, uint8_t row, uint8_t padlen)
{
  lcd_command(CMD_SET_ADDR + address_of(row, 0) + col);
  bool neg = false;
  uint8_t len = 0;
  uint8_t i;

  if (n == 0) {
    lcd_write_dram('0');
    for(i=1; i<padlen; i++)
      lcd_write_dram(' ');
    return;
  }

  unsigned char buf[10];
  if (n < 0) {
    n = -n;
    neg = true;
  }
  while (n > 0) {
    buf[len++] = (n % 10) + '0';
    n /= 10;
  }

  if(neg) {
    buf[len++] = '-';
  }

  for (i = len; i > 0; i--)
    lcd_write_dram(buf[i-1]);
  for (i = len; i < padlen; i++)
    lcd_write_dram(' ');
}


void lcd_clearScreen(void)
{
  lcd_command(CMD_CLEAR_SCREEN);
  delay_ms(2);
}
