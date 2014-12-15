/* Support for HD44780 LCD controller hooked in 4-bit mode via shift register 
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

