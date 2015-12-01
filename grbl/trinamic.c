#include <avr/io.h>
#include "config.h"
#include "trinamic.h"
#include "spi.h"

	#define CS_DDR				DDRB
	#define CS_PORT				PORTB
	#define X_CS_BIT			0 // Uno Digital Pin 8
	#define Y_CS_BIT			1 // Uno Digital Pin 9
	#define Z_CS_BIT			2 // Uno Digital Pin 10
	#define CS_MASK 			((1<<X_CS_BIT)|(1<<Y_CS_BIT)|(1<<Z_CS_BIT))


// global variable to keep stepper information
tos100 stepper_tos_100[3];


//some default values used in initialization
#define DEFAULT_MICROSTEPPING_VALUE 32

//TMC26X register definitions
#define DRIVER_CONTROL_REGISTER 0x0ul
#define CHOPPER_CONFIG_REGISTER 0x80000ul
#define COOL_STEP_REGISTER  0xA0000ul
#define STALL_GUARD2_LOAD_MEASURE_REGISTER 0xC0000ul
#define DRIVER_CONFIG_REGISTER 0xE0000ul

#define REGISTER_BIT_PATTERN 0xFFFFFul

//definitions for the driver control register
#define MICROSTEPPING_PATTERN 0xFul
#define STEP_INTERPOLATION 0x200ul
#define DOUBLE_EDGE_STEP 0x100ul
#define VSENSE 0x40ul
#define READ_MICROSTEP_POSTION 0x0ul
#define READ_STALL_GUARD_READING 0x10ul
#define READ_STALL_GUARD_AND_COOL_STEP 0x20ul
#define READ_SELECTION_PATTERN 0x30ul

//definitions for the chopper config register
#define CHOPPER_MODE_STANDARD 0x0ul
#define CHOPPER_MODE_T_OFF_FAST_DECAY 0x4000ul
#define T_OFF_PATTERN 0xful
#define RANDOM_TOFF_TIME 0x2000ul
#define BLANK_TIMING_PATTERN 0x18000ul
#define BLANK_TIMING_SHIFT 15
#define HYSTERESIS_DECREMENT_PATTERN 0x1800ul
#define HYSTERESIS_DECREMENT_SHIFT 11
#define HYSTERESIS_LOW_VALUE_PATTERN 0x780ul
#define HYSTERESIS_LOW_SHIFT 7
#define HYSTERESIS_START_VALUE_PATTERN 0x78ul
#define HYSTERESIS_START_VALUE_SHIFT 4
#define T_OFF_TIMING_PATERN 0xFul

//definitions for cool step register
#define MINIMUM_CURRENT_FOURTH 0x8000ul
#define CURRENT_DOWN_STEP_SPEED_PATTERN 0x6000ul
#define SE_MAX_PATTERN 0xF00ul
#define SE_CURRENT_STEP_WIDTH_PATTERN 0x60ul
#define SE_MIN_PATTERN 0xful

//definitions for stall guard2 current register
#define STALL_GUARD_FILTER_ENABLED 0x10000ul
#define STALL_GUARD_TRESHHOLD_VALUE_PATTERN 0x17F00ul
#define CURRENT_SCALING_PATTERN 0x1Ful
#define STALL_GUARD_CONFIG_PATTERN 0x17F00ul
#define STALL_GUARD_VALUE_PATTERN 0x7F00ul

//definitions for the input from the TCM260
#define STATUS_STALL_GUARD_STATUS 0x1ul
#define STATUS_OVER_TEMPERATURE_SHUTDOWN 0x2ul
#define STATUS_OVER_TEMPERATURE_WARNING 0x4ul
#define STATUS_SHORT_TO_GROUND_A 0x8ul
#define STATUS_SHORT_TO_GROUND_B 0x10ul
#define STATUS_OPEN_LOAD_A 0x20ul
#define STATUS_OPEN_LOAD_B 0x40ul
#define STATUS_STAND_STILL 0x80ul
#define READOUT_VALUE_PATTERN 0xFFC00ul

//default values
#define INITIAL_MICROSTEPPING 0x3ul //32th microstepping

//debuging output
//#define DEBUG

void TMC26XStepper_init( tos100 *tos100)
{
  //by default cool step is not enabled
  tos100->cool_step_enabled=false;
        
  //initialize register values
  tos100->driver_control_register_value=DRIVER_CONTROL_REGISTER | INITIAL_MICROSTEPPING;
  tos100->chopper_config_register=CHOPPER_CONFIG_REGISTER;
        
  //setting the default register values
  tos100->driver_control_register_value=DRIVER_CONTROL_REGISTER|INITIAL_MICROSTEPPING;
  tos100->microsteps = (1 << INITIAL_MICROSTEPPING);
  tos100->chopper_config_register=CHOPPER_CONFIG_REGISTER;
  tos100->cool_step_register_value=COOL_STEP_REGISTER;
  tos100->stall_guard2_current_register_value=STALL_GUARD2_LOAD_MEASURE_REGISTER;
  tos100->driver_configuration_register_value = DRIVER_CONFIG_REGISTER | READ_STALL_GUARD_READING;

  //set the current
  //TMC26XStepper_setCurrent(current, tos100);
  //set to a conservative start value
  //TMC26XStepper_setConstantOffTimeChopper(7, 54, 13,12,1, tos100);
  //set a nice microstepping value, TOO: broken!
  //TMC26XStepper_setMicrosteps(tos100->microsteps, tos100);
  
   TMC26XStepper_start(tos100);

   // after setup set pin to HIGH again
   CS_PORT |= _BV(tos100->cs_pin);
}

/*
 * start & configure the stepper driver
 * just must be called.
 */
void TMC26XStepper_start(tos100 *tos100) {
  //set the pins as output & its initial value
  
  // configure data direction bit and set pin to HIGH
  CS_DDR |= _BV(tos100->cs_pin);
  CS_PORT |= _BV(tos100->cs_pin);  
  
    
  //configure the SPI interface
  spi_setBitOrder(MSBFIRST);
  spi_setClockDivider(SPI_CLOCK_DIV8);
  //todo this does not work reliably - find a way to foolprof set it (e.g. while communicating
  //spi_setDataMode(SPI_MODE3);
  spi_begin();
                
  //set the initial values
  TMC26XStepper_send262(tos100->driver_control_register_value, tos100); 
  TMC26XStepper_send262(tos100->chopper_config_register, tos100);
  TMC26XStepper_send262(tos100->cool_step_register_value, tos100);
  TMC26XStepper_send262(tos100->stall_guard2_current_register_value, tos100);
  TMC26XStepper_send262(tos100->driver_configuration_register_value, tos100);
}

/*
 * send register settings to the stepper driver via SPI
 * returns the current status
 */
void TMC26XStepper_send262(unsigned long datagram, tos100 *tos100) {
  unsigned long i_datagram;
    
  //preserver the previous spi mode
  unsigned char oldMode =  SPCR & SPI_MODE_MASK;
        
  //if the mode is not correct set it to mode 3
  if (oldMode != SPI_MODE3) {
    spi_setDataMode(SPI_MODE3);
  }
        
  //select the TMC driver
  // set to LOW
  CS_PORT &= ~_BV(tos100->cs_pin);

  //ensure that only valid bist are set (0-19)
  //datagram &=REGISTER_BIT_PATTERN;
        
  //write/read the values
  i_datagram = spi_transfer((datagram >> 16) & 0xff);
  i_datagram <<= 8;
  i_datagram |= spi_transfer((datagram >>  8) & 0xff);
  i_datagram <<= 8;
  i_datagram |= spi_transfer((datagram) & 0xff);
  i_datagram >>= 4;
        
  //deselect the TMC chip
  //set to HIGH 
  CS_PORT |= _BV(tos100->cs_pin);
  
  //restore the previous SPI mode if neccessary
  //if the mode is not correct set it to mode 3
  if (oldMode != SPI_MODE3) {
    spi_setDataMode(oldMode);
  }

        
  //store the datagram as status result
  tos100->driver_status_result = i_datagram;
}

void TMC26XStepper_setCurrent(tos100 *tos100) {
  unsigned char current_scaling = 0;
  //calculate the current scaling from the max current setting (in mA)
  double mASetting = (double)tos100->current;
  double resistor_value = (double) tos100->resistor;
  // remove vesense flag
  tos100->driver_configuration_register_value &= ~(VSENSE); 
  //this is derrived from I=(cs+1)/32*(Vsense/Rsense)
  //leading to cs = CS = 32*R*I/V (with V = 0,31V oder 0,165V  and I = 1000*current)
  //with Rsense=0,15
  //for vsense = 0,310V (VSENSE not set)
  //or vsense = 0,165V (VSENSE set)
  current_scaling = (unsigned char)((resistor_value*mASetting*32.0/(0.31*1000.0*1000.0))-0.5); //theoretically - 1.0 for better rounding it is 0.5
        
  //check if the current scalingis too low
  if (current_scaling<16) {
    //set the csense bit to get a use half the sense voltage (to support lower motor currents)
    tos100->driver_configuration_register_value |= VSENSE;
    //and recalculate the current setting
    current_scaling = (unsigned char)((resistor_value*mASetting*32.0/(0.165*1000.0*1000.0))-0.5); //theoretically - 1.0 for better rounding it is 0.5
  }

  //do some sanity checks
  if (current_scaling>31) {
    current_scaling=31;
  }
  //delete the old value
  tos100->stall_guard2_current_register_value &= ~(CURRENT_SCALING_PATTERN);
  //set the new current scaling
  tos100->stall_guard2_current_register_value |= current_scaling;
  
  TMC26XStepper_send262(tos100->driver_configuration_register_value, tos100);
  TMC26XStepper_send262(tos100->stall_guard2_current_register_value, tos100);
  
}

/*
 * Set the number of microsteps per step.
 * 0,2,4,8,16,32,64,128,256 is supported
 * any value in between will be mapped to the next smaller value
 * 0 and 1 set the motor in full step mode
 */
void TMC26XStepper_setMicrosteps(int number_of_steps, tos100 *tos100) {
  long setting_pattern;
  //poor mans log
  if (number_of_steps>=256) {
    setting_pattern=0;

  } else if (number_of_steps>=128) {
    setting_pattern=1;

  } else if (number_of_steps>=64) {
    setting_pattern=2;

  } else if (number_of_steps>=32) {
    setting_pattern=3;

  } else if (number_of_steps>=16) {
    setting_pattern=4;

  } else if (number_of_steps>=8) {
    setting_pattern=5;

  } else if (number_of_steps>=4) {
    setting_pattern=6;

  } else if (number_of_steps>=2) {
    setting_pattern=7;

    //1 and 0 lead to full step
  } else if (number_of_steps<=1) {
    setting_pattern=8;

  }

  //delete the old value
  tos100->driver_control_register_value &=0xFFFF0ul;
  //set the new value
  tos100->driver_control_register_value |=setting_pattern;
        
  
  TMC26XStepper_send262(tos100->driver_control_register_value, tos100);
}

void TMC26XStepper_setConstantOffTimeChopper(char constant_off_time, char blank_time, char fast_decay_time_setting, char sine_wave_offset, unsigned char use_current_comparator, tos100 *tos100) {
  //perform some sanity checks
  if (constant_off_time<2) {
    constant_off_time=2;
  } else if (constant_off_time>15) {
    constant_off_time=15;
  }
  //save the constant off time
  tos100->constant_off_time = constant_off_time;
  char blank_value;
  //calculate the value acc to the clock cycles
  if (blank_time>=54) {
    blank_value=3;
  } else if (blank_time>=36) {
    blank_value=2;
  } else if (blank_time>=24) {
    blank_value=1;
  } else {
    blank_value=0;
  }
  if (fast_decay_time_setting<0) {
    fast_decay_time_setting=0;
  } else if (fast_decay_time_setting>15) {
    fast_decay_time_setting=15;
  }
  if (sine_wave_offset < -3) {
    sine_wave_offset = -3;
  } else if (sine_wave_offset>12) {
    sine_wave_offset = 12;
  }
  //shift the sine_wave_offset
  sine_wave_offset +=3;
        
  //calculate the register setting
  //first of all delete all the values for this
  tos100->chopper_config_register &= ~((1<<12) | BLANK_TIMING_PATTERN | HYSTERESIS_DECREMENT_PATTERN | HYSTERESIS_LOW_VALUE_PATTERN | HYSTERESIS_START_VALUE_PATTERN | T_OFF_TIMING_PATERN);
  //set the constant off pattern
  tos100->chopper_config_register |= CHOPPER_MODE_T_OFF_FAST_DECAY;
  //set the blank timing value
  tos100->chopper_config_register |= ((unsigned long)blank_value) << BLANK_TIMING_SHIFT;
  //setting the constant off time
  tos100->chopper_config_register |= constant_off_time;
  //set the fast decay time
  //set msb
  tos100->chopper_config_register |= (((unsigned long)(fast_decay_time_setting & 0x8))<<HYSTERESIS_DECREMENT_SHIFT);
  //other bits
  tos100->chopper_config_register |= (((unsigned long)(fast_decay_time_setting & 0x7))<<HYSTERESIS_START_VALUE_SHIFT);
  //set the sine wave offset
  tos100->chopper_config_register |= (unsigned long)sine_wave_offset << HYSTERESIS_LOW_SHIFT;
  //using the current comparator?
  if (!use_current_comparator) {
    tos100->chopper_config_register |= (1<<12);
  }

  //if started we directly send it to the motor
  TMC26XStepper_send262(tos100->driver_control_register_value, tos100);
 
}
