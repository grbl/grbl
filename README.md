## GRBL 0.9j for Arduino Mega 2560 and RAMPS 1.4

This port was initally developed by ArSi arsi@arsi.sk but has been enhanced to also support Arduino Mega 2560 + RAMPS 1.4 Board (including limit switches, homing and probing support).
It's based on Grbl v0.9j Atmega328p 16mhz 115200baud with generic defaults (2016-03-17).

For my home made CNC machine I have connected the limit switches as per the RAMPS 1.4 standard connection and not the grbl standard:
I.e. 
![grbl for ramps limit and probe connection](https://user-images.githubusercontent.com/942356/51804562-c217f480-2262-11e9-9f21-33e7b92b00d6.png)

This is the key configuration parameters for making both limit switching and probing working on my RAMPS 1.4 CNC machine:
[grbl RAMPS 1.4 config parameters](/grbl/defaults/defaults_pulpitrockcnc.h)
```
#define DEFAULT_INVERT_ST_ENABLE 0 // false
#define DEFAULT_INVERT_LIMIT_PINS 0 // false
// Uses software limits in the firmware to keep the printer from going too far in the opposite direction and hardware endstops for min
#define DEFAULT_SOFT_LIMIT_ENABLE 1 // true
#define DEFAULT_HARD_LIMIT_ENABLE 1 // true
#define DEFAULT_HOMING_ENABLE 1 // true
#define DEFAULT_HOMING_DIR_MASK ((1<<X_AXIS)|(1<<Y_AXIS)|(0<<Z_AXIS)) // X and Y endstop installed to the minimum (zero point). Z max installed and Z Min used for probe
```

On the RAMPS board the spindle direction output pins can be found under SERVOS
- SPINDLE_ENABLE_BIT = MEGA2560 Digital Pin 6
- SPINDLE_DIRECTION_BIT =  MEGA2560 Digital Pin 5

On the RAMPS board the flood and mist coolant enable output pins can be found under AUX-1
- COOLANT_FLOOD_BIT = MEGA2560 Digital Pin 0
- COOLANT_MIST_BIT = MEGA2560 Digital Pin 1

On the RAMPS board user-control CONTROLs (cycle start, reset, feed hold) input pins can be found under AUX-2
- RESET_BIT = MEGA2560 Analog Pin 9
- FEED_HOLD_BIT = MEGA2560 Analog Pin 10
- CYCLE_START_BIT = MEGA2560 Analog Pin 11
- SAFETY_DOOR_BIT = MEGA2560 Analog Pin 12

## Updated to use platformio

build using the following commands

```
$pio run
```
Upload to your board using

```
$pio run --target upload
```

***

Regards,
Per Ivar
