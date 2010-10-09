//*************************************************************************************
// include the library code:
#include <LiquidCrystal.h>
#include <WProgram.h>  //all things wiring / arduino
#include "config.h"
// 
// #define LCD_DB0 	4				// Using Ardiuno numbering, not port numbering
// #define LCD_DB1		5				// Equivalent to PORTD, pins 7 to 2
// #define LCD_DB2		6
// #define LCD_DB3		7
// #define LCD_ENABLE	3
// #define LCD_RS 		2


// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(2,3,4,5,6,7);//LCD_RS, LCD_ENABLE, LCD_DB0, LCD_DB1, LCD_DB2, LCD_DB3);
		
extern "C" int32_t actual_position[3];    // The current actual position of the tool in absolute steps
extern "C" int32_t position[3];    // The current target position of the tool in absolute steps

int pos_line=0;

extern "C" void lcd_report_position();
extern "C" void lcd_init();


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


void lcd_init() {
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


void lcd_report_position(void)
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

