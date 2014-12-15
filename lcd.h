/*
  lcd.h - Support for 20x4 LCD using HD44780 in 4-bit mode via shift register
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
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#define LCD_CLOCK_DDR DDRB
#define LCD_CLOCK_PORT PORTB
#define LCD_CLOCK_BIT 0 /* Digital Pin 8 (PB0) */

#define LCD_DATA_DDR DDRD
#define LCD_DATA_PORT PORTD
#define LCD_DATA_BIT 7 /* Digital Pin 7 (PD7) */

#define LCD_STROBE_DDR DDRD
#define LCD_STROBE_PORT PORTD
#define LCD_STROBE_BIT 4 /* Digital Pin 4 (PD4) */

/* This defines how the LCD controller is wired to the shift register

   Upper data bits D7-D4 can be transmitted as-is to the controller on bits 0-3.
   Being in 4-bit mode, first data bits are the high nibble then the low nibble.
 */
#define SHIFT_RS 6 /* Q2 */
#define SHIFT_E  4 /* Q4 */
#define SHIFT_RW 5 /* Q3 */

void lcd_init(void);
void lcd_clearScreen(void);

void lcd_latestCommand(char *cmd);
void lcd_updateAxes(void);

