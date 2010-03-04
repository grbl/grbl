Grbl - An embedded rs274/ngc (g-code) integrater interpreter and motion-controller for the Arduino/AVR328 microcontroller

Goal: A no-compromise, high performance, low cost alternative to parallel-port based motion control for CNC milling

Status:
* Ready for production, but probably rough around the edges still
* Highly optimized C utilizing the hardware-timers of the AVR-chip for all critical timing
* Able to maintain more than 30kHz step rate, generating an ultra clean, jitter free step-signal
* G-code interpreter complete, tested with output from several CAM tools
* Standards-compliant g-code arcs/circles fully supported
* Buffered, non blocking, asynchronous step generation so the rest of the system is free to process
  g-code while the steppers are steppin'
* Tested on very few (two) CNC rigs

Pending: 
* Battle hardening in the field
* Documentation and web-site
* Simpler configuration (w/o recompilation)
* Optional support for a alphanumeric LCD readout, a joystick and a few buttons for program control
* Support "headless" fabrication by buffering all code to SD-card or similar
* Easing of feed rate
* Spindle control

Limitations by design:
* Limited GCode-support. Focus on the kind of GCode produced by CAM tools. Leave human GCoders frustrated.
* No support for tool offsets (typically handled by CAM-tool)
* No rotation axes, only x, y and z.

The project was initially inspired by the Arduino GCode Interpreter by Mike Ellery
