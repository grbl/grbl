Grbl - an open source, embedded high performance g-code-parser and CNC milling controller written in optimized C that 
will run on a straight Arduino.

Documentation: http://dank.bengler.no/-/page/show/5470_grbl
Dev announcments on twitter @grblcnc: http://twitter.com/grblcnc

Status:
* Ready for production, but probably rough around the edges still
* Highly optimized C utilizing the hardware-timers of the AVR-chip for all critical timing
* Able to maintain more than 30kHz step rate, generating an ultra clean, jitter free step-signal
* G-code interpreter complete, tested with output from several CAM tools
* Standards-compliant g-code arcs/circles fully supported
* Buffered, non blocking, asynchronous step generation so the rest of the system is free to process
  g-code while the steppers are steppin'
* Configuration parameters stored in EEPROM and set via simple commands
* Tested on very few (two) CNC rigs

Limitations by design:
* Limited GCode-support. Focus on the kind of GCode produced by CAM tools. Leave human GCoders frustrated.
* No support for tool offsets (typically handled by CAM-tool)
* No rotation axes, only x, y and z.

Prioritized to-do:
* Accelleration/decelleration
* Spindle control
* Autodetect baud rate
* Arduino IDE compatible (build and flash)
* Documentation and web-site
* Support for a alphanumeric LCD readout, a joystick and a few buttons for program control
* Support "headless" fabrication by buffering all code to SD-card or similar

The project was initially inspired by the Arduino GCode Interpreter by Mike Ellery
