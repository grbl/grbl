#ifndef config_default_2560_h
#define config_default_2560_h

// Define pin-assignments
// NOTE: All step bit and direction pins must be on the same port.
#define STEPPING_DDR      DDRA
#define STEPPING_PORT     PORTA
#define STEPPING_PIN      PINA
#define X_STEP_BIT        0 // MEGA2560 Digital Pin 22
#define Y_STEP_BIT        2 // MEGA2560 Digital Pin 24
#define Z_STEP_BIT        4 // MEGA2560 Digital Pin 26
#define C_STEP_BIT        6 // MEGA2560 Digital Pin 28
#define X_DIRECTION_BIT   1 // MEGA2560 Digital Pin 23
#define Y_DIRECTION_BIT   3 // MEGA2560 Digital Pin 25
#define Z_DIRECTION_BIT   5 // MEGA2560 Digital Pin 27
#define C_DIRECTION_BIT   7 // MEGA2560 Digital Pin 29
#define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits
#define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits
#define STEPPING_MASK (STEP_MASK | DIRECTION_MASK) // All stepping-related bits (step/direction)

#define STEPPERS_DISABLE_DDR   DDRC
#define STEPPERS_DISABLE_PORT  PORTC
#define STEPPERS_DISABLE_BIT   7 // MEGA2560 Digital Pin 30
#define STEPPERS_DISABLE_MASK (1<<STEPPERS_DISABLE_BIT)

// NOTE: All limit bit pins must be on the same port
#define LIMIT_DDR       DDRC
#define LIMIT_PORT      PORTC
#define LIMIT_PIN       PINC
#define X_LIMIT_BIT     6 // MEGA2560 Digital Pin 31
#define Y_LIMIT_BIT     5 // MEGA2560 Digital Pin 32
#define Z_LIMIT_BIT     4 // MEGA2560 Digital Pin 33
#define C_LIMIT_BIT     3 // MEGA2560 Digital Pin 34
#define LIMIT_INT       PCIE0  // Pin change interrupt enable pin
#define LIMIT_INT_vect  PCINT0_vect 
#define LIMIT_PCMSK     PCMSK0 // Pin change interrupt register
#define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits

#define SPINDLE_ENABLE_DDR   DDRC
#define SPINDLE_ENABLE_PORT  PORTC
#define SPINDLE_ENABLE_BIT   2 // MEGA2560 Digital Pin 35

#define SPINDLE_DIRECTION_DDR   DDRC
#define SPINDLE_DIRECTION_PORT  PORTC
#define SPINDLE_DIRECTION_BIT   1 // MEGA2560 Digital Pin 36

#define COOLANT_FLOOD_DDR   DDRC
#define COOLANT_FLOOD_PORT  PORTC
#define COOLANT_FLOOD_BIT   0 // MEGA2560 Digital Pin 37

// NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
// a later date if flash and memory space allows.
// #define ENABLE_M7  // Mist coolant disabled by default. Uncomment to enable.
#ifdef ENABLE_M7
  #define COOLANT_MIST_DDR   DDRC
  #define COOLANT_MIST_PORT  PORTC
  #define COOLANT_MIST_BIT   0 // MEGA2560 Digital Pin 37
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
