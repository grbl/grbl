#include <WProgram.h>  //all things wiring / arduino
#include <Wire.h>
//#include "config.h"

extern "C" void i2c_init();
extern "C" void i2c_write_value(byte addr, byte config, int32_t value);
extern "C" void i2c_report_position(void);
extern "C" void i2c_get_buttons(void);

//extern TwoWire Wire;

extern "C" int32_t actual_position[3];    // The current actual position of the tool in absolute steps
extern "C" int32_t position[3];    // The current target position of the tool in absolute steps
char buttons[4];

int i2c_line=0;


void i2c_init()
{
  Wire.begin(); // join i2c bus (address optional for master)
}

void i2c_write_value(byte addr, byte config, int32_t value)
{
  Wire.beginTransmission(addr); 	
  Wire.send(config);
  Wire.send((char)((value >> 24) & 0xFF));      
  Wire.send((char)((value >> 16) & 0xFF));      
  Wire.send((char)((value >> 8) & 0xFF));      
  Wire.send((char)(value & 0xFF));      
  Wire.endTransmission();    	
}


void i2c_report_position(void)
{
	if (i2c_line<3){
		i2c_write_value(4,128+i2c_line,actual_position[i2c_line]);
	} else {
		i2c_write_value(4,128+i2c_line,position[i2c_line-3]);
	}
	i2c_line++;
	if (i2c_line>5) i2c_line=0;
}


void i2c_get_buttons(void)
{
  Wire.requestFrom(4, 4);    // request 4 bytes from slave device #4

  char i;

  for(i=0;i<4;i++){
	if (Wire.available()) {
  		buttons[i] = Wire.receive();
  	}
  }
}