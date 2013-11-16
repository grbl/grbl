/*
  pin_map.h - Pin mapping configuration file
  Part of Grbl

  Copyright (c) 2013 Sungeun K. Jeon

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

/* The pin_map.h file serves as a central pin mapping settings file for different processor
   types, i.e. AVR 328p or AVR Mega 2560. Grbl officially supports the Arduino Uno, but the 
   other supplied pin mappings are supplied by users, so your results may vary. */

#ifndef pin_map_h
#define pin_map_h

#ifdef PIN_MAP_ARDUINO_UNO // AVR 328p, Officially supported by Grbl.

  // Serial port pins
  #define SERIAL_RX USART_RX_vect
  #define SERIAL_UDRE USART_UDRE_vect

  // Start of PWM & Stepper Enabled Spindle
  // CURRENTLY IN TESTING USE WITH CAUTION @EliteEng
  #define VARIABLE_SPINDLE // comment this out to disable PWM & Stepper on the spindle

  // NOTE: All step bit and direction pins must be on the same port.
  #define STEPPING_DDR       DDRD
  #define STEPPING_PORT      PORTD
  #define X_STEP_BIT         2  // Uno Digital Pin 2
  #define Y_STEP_BIT         3  // Uno Digital Pin 3
  #define Z_STEP_BIT         4  // Uno Digital Pin 4
  #define X_DIRECTION_BIT    5  // Uno Digital Pin 5
  #define Y_DIRECTION_BIT    6  // Uno Digital Pin 6
  #define Z_DIRECTION_BIT    7  // Uno Digital Pin 7
  #define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits
  #define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits
  #define STEPPING_MASK (STEP_MASK | DIRECTION_MASK) // All stepping-related bits (step/direction)

  #define STEPPERS_DISABLE_DDR    DDRB
  #define STEPPERS_DISABLE_PORT   PORTB
  #define STEPPERS_DISABLE_BIT    0  // Uno Digital Pin 8
  #define STEPPERS_DISABLE_MASK (1<<STEPPERS_DISABLE_BIT)

#ifdef VARIABLE_SPINDLE // Y LImit has been disabled to make room for the Spindle PWM
  // NOTE: All limit bit pins must be on the same port
  #define LIMIT_DDR       DDRB
  #define LIMIT_PIN       PINB
  #define LIMIT_PORT      PORTB
  #define X_LIMIT_BIT     1  // Uno Digital Pin 9
  #define Y_LIMIT_BIT     1  // Uno Digital Pin 9
  #define Z_LIMIT_BIT     3  // Uno Digital Pin 11
  #define LIMIT_INT       PCIE0  // Pin change interrupt enable pin
  #define LIMIT_INT_vect  PCINT0_vect 
  #define LIMIT_PCMSK     PCMSK0 // Pin change interrupt register
  #define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits

#else
  // NOTE: All limit bit pins must be on the same port
  #define LIMIT_DDR       DDRB
  #define LIMIT_PIN       PINB
  #define LIMIT_PORT      PORTB
  #define X_LIMIT_BIT     1  // Uno Digital Pin 9
  #define Y_LIMIT_BIT     2  // Uno Digital Pin 10
  #define Z_LIMIT_BIT     3  // Uno Digital Pin 11
  #define LIMIT_INT       PCIE0  // Pin change interrupt enable pin
  #define LIMIT_INT_vect  PCINT0_vect 
  #define LIMIT_PCMSK     PCMSK0 // Pin change interrupt register
  #define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits

#endif

// WARNING if INVERT_SPINDLE is defined the pin will be LOW(Active) on reset of controller until spindle_init() is called.
// this should only be a very short period of time but will energise the spindle.
// #define INVERT_SPINDLE // This will INVERT the Spindle LOW=Active
// END WARNING

  #define SPINDLE_ENABLE_DDR   DDRB
  #define SPINDLE_ENABLE_PORT  PORTB
  #define SPINDLE_ENABLE_BIT   4  // Uno Digital Pin 12

  #define SPINDLE_DIRECTION_DDR   DDRB
  #define SPINDLE_DIRECTION_PORT  PORTB
  #define SPINDLE_DIRECTION_BIT   5  // Uno Digital Pin 13 (NOTE: D13 can't be pulled-high input due to LED.)

  #define COOLANT_FLOOD_DDR   DDRC
  #define COOLANT_FLOOD_PORT  PORTC
  #define COOLANT_FLOOD_BIT   3  // Uno Analog Pin 3

  // NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
  // a later date if flash and memory space allows.
  // #define ENABLE_M7  // Mist coolant disabled by default. Uncomment to enable.
  #ifdef ENABLE_M7
    #define COOLANT_MIST_DDR   DDRC
    #define COOLANT_MIST_PORT  PORTC
    #define COOLANT_MIST_BIT   4 // Uno Analog Pin 4
  #endif  

  // NOTE: All pinouts pins must be on the same port
  #define PINOUT_DDR       DDRC
  #define PINOUT_PIN       PINC
  #define PINOUT_PORT      PORTC
  #define PIN_RESET        0  // Uno Analog Pin 0
  #define PIN_FEED_HOLD    1  // Uno Analog Pin 1
  #define PIN_CYCLE_START  2  // Uno Analog Pin 2
  #define PINOUT_INT       PCIE1  // Pin change interrupt enable pin
  #define PINOUT_INT_vect  PCINT1_vect
  #define PINOUT_PCMSK     PCMSK1 // Pin change interrupt register
  #define PINOUT_MASK ((1<<PIN_RESET)|(1<<PIN_FEED_HOLD)|(1<<PIN_CYCLE_START))
  

#ifdef VARIABLE_SPINDLE

//#define SPINDLE_IS_STEPPER // Uncomment this if your Spindle is a Stepper Motor
#define SPINDLE_IS_PWM // Uncomment this if your Spindle uses PWM ie. DC Motor

    #ifdef SPINDLE_IS_PWM
      #define SPINDLE_MAX_RPM 255 // Max RPM of your spindle - This value is equal to 100% Duty Cycle on the PWM
    #endif

	#ifdef SPINDLE_IS_STEPPER
      #define SPINDLE_STEPS_PER_REV	200 // if you have micro stepping enabled you need to calculate that to
	#endif

// Advanced Configuration Below You should not need to touch these variables

//Set Timer up to use TIMER4 OCR4B which is attached to Digital Pin 7
#define TCCRA_REGISTER TCCR1A
#define TCCRB_REGISTER TCCR1B
#define OCRA_REGISTER OCR1A
#define OCRB_REGISTER OCR1B

#define COMB_BIT COM1B1
#define WAVE0_REGISTER WGM10
#define WAVE1_REGISTER WGM11
#define WAVE2_REGISTER WGM12
#define WAVE3_REGISTER WGM13

  #define SPINDLE_PWM_DDR   DDRB
  #define SPINDLE_PWM_PORT  PORTB
  #define SPINDLE_PWM_BIT   2 // UNO Digital Pin 10

 #endif // End of VARIABLE_SPINDLE
#endif


#ifdef PIN_MAP_ARDUINO_MEGA_2560 // Working @EliteEng - Currently testing Variable Spindle

/* To Change the settings to usse the MEGA2560 you need to do the following
 * - Change config.h line 39 to #define PIN_MAP_ARDUINO_MEGA_2560
 * - Change Makefile line 31 to DEVICE     ?= atmega2560
 */

  // Serial port pins
  #define SERIAL_RX USART0_RX_vect
  #define SERIAL_UDRE USART0_UDRE_vect

  // Increase Buffers to make use of extra SRAM
  //#define RX_BUFFER_SIZE 256
  //#define TX_BUFFER_SIZE 128
  //#define BLOCK_BUFFER_SIZE 36
  //#define LINE_BUFFER_SIZE 100

  // NOTE: All step bit and direction pins must be on the same port.
  #define STEPPING_DDR      DDRA
  #define STEPPING_PORT     PORTA
  #define STEPPING_PIN      PINA
  #define X_STEP_BIT        2 // MEGA2560 Digital Pin 24
  #define Y_STEP_BIT        3 // MEGA2560 Digital Pin 25
  #define Z_STEP_BIT        4 // MEGA2560 Digital Pin 26
  #define X_DIRECTION_BIT   5 // MEGA2560 Digital Pin 27
  #define Y_DIRECTION_BIT   6 // MEGA2560 Digital Pin 28
  #define Z_DIRECTION_BIT   7 // MEGA2560 Digital Pin 29
  #define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits
  #define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits
  #define STEPPING_MASK (STEP_MASK | DIRECTION_MASK) // All stepping-related bits (step/direction)

  #define STEPPERS_DISABLE_DDR   DDRB
  #define STEPPERS_DISABLE_PORT  PORTB
  #define STEPPERS_DISABLE_BIT   7 // MEGA2560 Digital Pin 13
  #define STEPPERS_DISABLE_MASK (1<<STEPPERS_DISABLE_BIT)

  // NOTE: All limit bit pins must be on the same port
  #define LIMIT_DDR       DDRB
  #define LIMIT_PORT      PORTB
  #define LIMIT_PIN       PINB
  #define X_LIMIT_BIT     4 // MEGA2560 Digital Pin 10
  #define Y_LIMIT_BIT     5 // MEGA2560 Digital Pin 11
  #define Z_LIMIT_BIT     6 // MEGA2560 Digital Pin 12
  #define LIMIT_INT       PCIE0  // Pin change interrupt enable pin
  #define LIMIT_INT_vect  PCINT0_vect 
  #define LIMIT_PCMSK     PCMSK0 // Pin change interrupt register
  #define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits

// WARNING if INVERT_SPINDLE is defined the pin will be LOW(Active) on reset of controller until spindle_init() is called.
// this should only be a very short period of time but will energise the spindle.
// #define INVERT_SPINDLE // This will INVERT the Spindle LOW=Active
// END WARNING

  #define SPINDLE_ENABLE_DDR   DDRH
  #define SPINDLE_ENABLE_PORT  PORTH
  #define SPINDLE_ENABLE_BIT   3 // MEGA2560 Digital Pin 6

  #define SPINDLE_DIRECTION_DDR   DDRE
  #define SPINDLE_DIRECTION_PORT  PORTE
  #define SPINDLE_DIRECTION_BIT   3 // MEGA2560 Digital Pin 5

  #define COOLANT_FLOOD_DDR   DDRH
  #define COOLANT_FLOOD_PORT  PORTH
  #define COOLANT_FLOOD_BIT   5 // MEGA2560 Digital Pin 8

  // #define ENABLE_M7  // Mist coolant disabled by default. Uncomment to enable.
  #ifdef ENABLE_M7
    #define COOLANT_MIST_DDR   DDRH
    #define COOLANT_MIST_PORT  PORTH
    #define COOLANT_MIST_BIT   6 // MEGA2560 Digital Pin 9
  #endif  

  // NOTE: All pinouts pins must be on the same port
  #define PINOUT_DDR       DDRK
  #define PINOUT_PIN       PINK
  #define PINOUT_PORT      PORTK
  #define PIN_RESET        0  // MEGA2560 Analog Pin 8
  #define PIN_FEED_HOLD    1  // MEGA2560 Analog Pin 9
  #define PIN_CYCLE_START  2  // MEGA2560 Analog Pin 10
  #define PINOUT_INT       PCIE2  // Pin change interrupt enable pin
  #define PINOUT_INT_vect  PCINT2_vect
  #define PINOUT_PCMSK     PCMSK2 // Pin change interrupt register
  #define PINOUT_MASK ((1<<PIN_RESET)|(1<<PIN_FEED_HOLD)|(1<<PIN_CYCLE_START))

// Start of PWM & Stepper Enabled Spindle
 #define VARIABLE_SPINDLE // comment this out to disable PWM & Stepper on the spindle
#ifdef VARIABLE_SPINDLE

//#define SPINDLE_IS_STEPPER // Uncomment this if your Spindle is a Stepper Motor
#define SPINDLE_IS_PWM // Uncomment this if your Spindle uses PWM ie. DC Motor

    #ifdef SPINDLE_IS_PWM
      #define SPINDLE_MAX_RPM 255 // Max RPM of your spindle
    #endif

	#ifdef SPINDLE_IS_STEPPER
      #define SPINDLE_STEPS_PER_REV	200 // if you have micro stepping enabled you need to calculate that to
	#endif

// Advanced Configuration Below You should not need to touch these variables

//Set Timer up to use TIMER4 OCR4B which is attached to Digital Pin 7
#define TCCRA_REGISTER TCCR4A
#define TCCRB_REGISTER TCCR4B
#define OCRA_REGISTER OCR4A
#define OCRB_REGISTER OCR4B

#define COMB_BIT COM4B1
#define WAVE0_REGISTER WGM40
#define WAVE1_REGISTER WGM41
#define WAVE2_REGISTER WGM42
#define WAVE3_REGISTER WGM43

  #define SPINDLE_PWM_DDR   DDRH
  #define SPINDLE_PWM_PORT  PORTH
  #define SPINDLE_PWM_BIT   4 // MEGA2560 Digital Pin 7


 #endif // End of VARIABLE_SPINDLE

#endif

/* 
#ifdef PIN_MAP_CUSTOM_PROC
  // For a custom pin map or different processor, copy and paste one of the default pin map
  // settings above and modify it to your needs. Then, make sure the defined name is also
  // changed in the config.h file.
#endif
*/

#endif
