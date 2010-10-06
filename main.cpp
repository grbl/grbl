/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
  Part of Grbl

  Copyright (c) 2009 Simen Svale Skogsrud

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

extern "C" {
#include <avr/io.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "stepper.h"
#include "spindle_control.h"
#include "motion_control.h"
#include "gcode.h"
#include "config.h"
#include "wiring_serial.h"
#include "serial_protocol.h"
}

#include <WProgram.h>  //all things wiring / arduino

extern "C" int32_t actual_position[3];    // The current actual position of the tool in absolute steps
extern "C" int32_t position[3];    // The current target position of the tool in absolute steps

int pos_line=0;

//*************************************************************************************
// include the library code:
#include <LiquidCrystal.h>

#define LCD_DB0 	4				// Using Ardiuno numbering, not port numbering
#define LCD_DB1		5				// Equivalent to PORTD, pins 7 to 2
#define LCD_DB2		6
#define LCD_DB3		7
#define LCD_ENABLE	3
#define LCD_RS 		2


// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(LCD_RS, LCD_ENABLE, LCD_DB0, LCD_DB1, LCD_DB2, LCD_DB3);
										

extern "C" void lcd_print_char(char character);
extern "C" void lcd_print_long(long num);
extern "C" void lcd_print_str(char * line);


void lcd_print_char(char character)
{
	lcd.print(character);
}

void lcd_print_long(long num)
{
	lcd.print(num);
}
void lcd_print_str(char * line)
{
	lcd.print(line);
}


void setup_lcd() {
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print('X');
  lcd.setCursor(8,0);
  lcd.print("Z");
  lcd.setCursor(0,1);
  lcd.print("Y");
  
  pinMode(13,OUTPUT);
  digitalWrite(13, HIGH);
}


void lcd_print_count_as_mm(float count)
{
	// Convert position in steps to mm:
	// pos = steps * 1.27/1600 
	//     = steps * .00079375
	//     = steps * 79375 / 100 000 000    (1e-8) 
	
	long whole;
	long fraction;
	float answer;
	
	answer = count*0.79375;
	whole = round(answer/10.0);
	fraction = abs(whole) % 100;
	whole = whole/100;
	if (whole>=0) lcd.print(" "); // Allow space for - if required
	if (abs(whole)<100) lcd.print(" ");
	if (abs(whole)<10) lcd.print(" ");
	lcd.print(whole);
	lcd.print(".");
	if (fraction<10) lcd.print("0");
	lcd.print(fraction);
} 


void report_position(void)
{
// Only report 1 of X, Y or Z position per time, allows time for other
// stuff to happen (ie doesn't hog the main thread for too long.

	switch (pos_line){
	case 0:  lcd.setCursor(1, 0);
			 lcd_print_count_as_mm(actual_position[0]);
			 break;
	case 1:  lcd.setCursor(1, 1);
			 lcd_print_count_as_mm(actual_position[1]);
			 break;
	case 2:  lcd.setCursor(10, 0);
			 lcd_print_count_as_mm(actual_position[2]);
			 break;
	}
	pos_line++;
	if (pos_line>2) pos_line=0;
}



//*************************************************************************************
int main(void)
{
	setup_lcd();
//	loop();



  beginSerial(BAUD_RATE);
  config_init();
  st_init(); // initialize the stepper subsystem
  mc_init(); // initialize motion control subsystem
  spindle_init(); // initialize spindle controller
  gc_init(); // initialize gcode-parser
  sp_init(); // initialize the serial protocol
  
  DDRD |= (1<<3)|(1<<4)|(1<<5);
  
  for(;;){
    report_position();
    //sleep_mode();
    sp_process(); // process the serial protocol
  }
  return 0;   /* never reached */
}
