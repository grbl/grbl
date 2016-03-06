/*
  cpu_map_atmega2560_ramps14.h - CPU and pin mapping configuration file
  Part of Grbl

  Copyright (c) 2012-2015 Sungeun K. Jeon

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

/* This cpu_map file serves as a central pin mapping settings file for AVR Mega 2560 + RAMPS 1.4 
   PIN Mapping can be found here: https://www.arduino.cc/en/Hacking/PinMapping2560
*/


#ifdef GRBL_PLATFORM
#error "cpu_map already defined: GRBL_PLATFORM=" GRBL_PLATFORM
#endif


#define GRBL_PLATFORM "Atmega2560_Ramps14"

// Serial port pins
#define SERIAL_RX USART0_RX_vect
#define SERIAL_UDRE USART0_UDRE_vect

// Increase Buffers to make use of extra SRAM
//#define RX_BUFFER_SIZE		256
//#define TX_BUFFER_SIZE		128
//#define BLOCK_BUFFER_SIZE	36
//#define LINE_BUFFER_SIZE	100

// Define step pulse output pins. 
// NOTE: Originally grbl require that all step bit pins must be on the same port.
// HOWEVER: X and Y is on same port while Z is on another, therefore a special method from ramps.h is needed
#define X_STEP_BIT    0 // Position within the STEP MASK
#define Y_STEP_BIT    1 // Position within the STEP MASK
#define Z_STEP_BIT    2 // Position within the STEP MASK
#define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits

// Define step direction output pins. 
// NOTE: Originally grbl require that all direction pins must be on the same port
// HOWEVER X and Y is on same port while Z is on another, therefore a special method from ramps.h is needed
#define X_DIRECTION_BIT   0 // Position within the DIRECTION MASK
#define Y_DIRECTION_BIT   1 // Position within the DIRECTION MASK
#define Z_DIRECTION_BIT   2 // Position within the DIRECTION MASK
#define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits

// NOTE: Originally grbl require an enable/disable output pin for the stepper driver.
// HOWEVER: None of the driver enable pins are on the same port, therefore a special method from ramps.h is needed
#define STEPPERS_DISABLE_BIT   0 // A bit position used to determine if the steppers are enabled or disabled, any number should work

// Define homing/hard limit switch input pins and limit interrupt vectors. 
// NOTE: Originally grbl require that all limit bit pins must be on the same port 
// HOWEVER: None of them are! Therefore a special method from ramps.h is needed
// MEGA2560 Digital Pin 3,  X_MIN_PIN, PE5 ( OC3C/INT5 )
// MEGA2560 Digital Pin 14, Y_MIN_PIN, PJ1 ( TXD3/PCINT10 )
// MEGA2560 Digital Pin 19, Z_MAX_PIN, PD2 ( RXDI/INT2 )
#define X_LIMIT_BIT     2 // Position within the LIMIT MASK
#define Y_LIMIT_BIT     1 // Position within the LIMIT MASK
#define Z_LIMIT_BIT     0 // Position within the LIMIT MASK
#define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits

// MEGA2560 Digital Pin 3,  X_MIN_PIN, PE5 ( OC3C/INT5 )
// X_MIN uses external interrupts
#define X_MIN_INT      INT5  	 // INTx bit in EIMSK 
#define X_MIN_ICR      EICRB 	 // Int. Config Register (EICRA/B) 
#define X_MIN_ISCX0    ISC50 	 // Interrupt Sense Config bit0 
#define X_MIN_ISCX1    ISC51 	 // Interrupt Sense Config bit1 
#define X_MIN_INT_vect INT5_vect // Pin Change Interrupt Vector
#define X_MIN_FR       INTF5 	 // Interrupt Flag Register

// MEGA2560 Digital Pin 19, Z_MAX_PIN, PD2 ( RXDI/INT2 )
// Z_MAX uses external interrupts
#define Z_MAX_INT      INT2  	 // INTx bit in EIMSK 
#define Z_MAX_ICR      EICRA 	 // Int. Config Register (EICRA/B) 
#define Z_MAX_ISCX0    ISC20 	 // Interrupt Sense Config bit0 
#define Z_MAX_ISCX1    ISC21 	 // Interrupt Sense Config bit1 
#define Z_MAX_INT_vect INT2_vect // Pin Change Interrupt Vector
#define Z_MAX_FR       INTF2 	 // Interrupt Flag Register

// MEGA2560 Digital Pin 14, Y_MIN_PIN, PJ1 ( TXD3/PCINT10 )
// Y_MIN uses pin change interrupts
#define Y_MIN_PCI       PCINT10     // Pin Change Interrupt
#define Y_MIN_INT       PCIE1       // Pin Change Interrupt Enable Pin
// PCIE0 = Any change on any enabled PCINT7:0 pin will cause an interrupt (Port B)
// PCIE1 = Any change on any enabled PCINT15:8 pin will cause an interrupt (Port J)
// PCIE2 = Any change on any enabled PCINT23:16 pin will cause an interrupt (Port K)
#define Y_MIN_INT_vect  PCINT1_vect // Pin Change Interrupt Vector
#define Y_MIN_PCMSK     PCMSK1      // Pin Change Mask Register
#define Y_MIN_FR        PCIF1       // Pin Change Interrupt Flag Register

// Define spindle enable and spindle direction output pins.
// On the RAMPS board these can be found under SERVOS
#define SPINDLE_ENABLE_DDR      DDRH
#define SPINDLE_ENABLE_PORT     PORTH
#define SPINDLE_ENABLE_BIT      3 // MEGA2560 Digital Pin 6, PH3 ( OC4A )
#define SPINDLE_DIRECTION_DDR   DDRE
#define SPINDLE_DIRECTION_PORT  PORTE
#define SPINDLE_DIRECTION_BIT   3 // MEGA2560 Digital Pin 5, PE3 ( OC3A/AIN1 )

// Define flood and mist coolant enable output pins.
// NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
// a later date if flash and memory space allows.
// On the RAMPS board these can be found under AUX-1
#define COOLANT_FLOOD_DDR     DDRE
#define COOLANT_FLOOD_PORT    PORTE
#define COOLANT_FLOOD_BIT     0 // MEGA2560 Digital Pin 0, PE0 ( RXD0/PCINT8 )
#ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
  #define COOLANT_MIST_DDR      DDRE
  #define COOLANT_MIST_PORT     PORTE
  #define COOLANT_MIST_BIT      1 // MEGA2560 Digital Pin 1, PE1 ( TXD0 )
#endif  

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
// NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
// On the RAMPS board these can be found under AUX-2
#define CONTROL_DDR       DDRK
#define CONTROL_PIN       PINK
#define CONTROL_PORT      PORTK
#define RESET_BIT         1  // MEGA2560 Analog Pin 9, PK1 ( ADC9/PCINT17 )
#define FEED_HOLD_BIT     2  // MEGA2560 Analog Pin 10, PK2 ( ADC10/PCINT18 )
#define CYCLE_START_BIT   3  // MEGA2560 Analog Pin 11, PK3 ( ADC11/PCINT19 )
#define SAFETY_DOOR_BIT   4  // MEGA2560 Analog Pin 12, PK4 ( ADC12/PCINT20 )
#define CONTROL_INT       PCIE2  // Pin change interrupt enable pin
#define CONTROL_INT_vect  PCINT2_vect
#define CONTROL_PCMSK     PCMSK2 // Pin change interrupt register
#define CONTROL_MASK ((1<<RESET_BIT)|(1<<FEED_HOLD_BIT)|(1<<CYCLE_START_BIT)|(1<<SAFETY_DOOR_BIT))
#define CONTROL_INVERT_MASK CONTROL_MASK // May be re-defined to only invert certain control pins.

// Define probe switch input pin.
// On the RAMPS board the Z probe (touch plate) is connected to the Z_MIN pin (D18)
#define PROBE_DDR       DDRD
#define PROBE_PIN       PIND
#define PROBE_PORT      PORTD
#define PROBE_BIT       3  // MEGA2560 Digital Pin 18, PD3 ( TXD1/INT3 )
#define PROBE_MASK      (1<<PROBE_BIT)

// Start of PWM & Stepper Enabled Spindle
#ifdef VARIABLE_SPINDLE
  // Advanced Configuration Below You should not need to touch these variables
  // Set Timer up to use TIMER4B which is attached to Digital Pin 7
  #define PWM_MAX_VALUE     65535.0
  #define TCCRA_REGISTER    TCCR4A
  #define TCCRB_REGISTER    TCCR4B
  #define OCR_REGISTER      OCR4B
  
  #define COMB_BIT          COM4B1
  #define WAVE0_REGISTER    WGM40
  #define WAVE1_REGISTER    WGM41
  #define WAVE2_REGISTER    WGM42
  #define WAVE3_REGISTER    WGM43
  
  #define SPINDLE_PWM_DDR   DDRH
  #define SPINDLE_PWM_PORT  PORTH
  #define SPINDLE_PWM_BIT   4 // MEGA2560 Digital Pin 7, PH4 ( OC4B )
#endif // End of VARIABLE_SPINDLE

// RAMPS 1.4 Pin assignments
// Taken from Marlin pins.h (BOARD_RAMPS_13_EFF) and http://reprap.org/wiki/RAMPS_1.4
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_MIN_PIN           3
#define X_MAX_PIN           2

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24

#define E1_STEP_PIN        36
#define E1_DIR_PIN         34
#define E1_ENABLE_PIN      30

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13

#define FAN_PIN             9

#define PS_ON_PIN          12
#define KILL_PIN           -1

#define HEATER_0_PIN       10
#define HEATER_1_PIN        8
#define TEMP_0_PIN         13   // ANALOG NUMBERING
#define TEMP_1_PIN         14   // ANALOG NUMBERING
#define TEMP_2_PIN         15   // ANALOG NUMBERING
