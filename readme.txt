Grbl - An embedded rs274/ngc (g-code) interpreter, CNC controller, readout and exerciser for the AVR series of microcontrollers.
Inspired by the Arduino GCode Interpreter by Mike Ellery

Goals: 
* Runs on the atmega168
* Runs on a standard Arduino
* Suitable for both milling and deposition fabrication (e.g. RepRap)
* Support GCode from common free and cheap CAM-packages right out of the box
* Optional support for a alphanumeric LCD readout, a joystick and a few buttons for program control
* Optional support for automated cutter length calibration when milling
* Support "headless" fabrication by buffering all code to SD-card or similar

Status:
* Runs on atmega168/arduino.
* GCode interpreter complete
* Linear interpolation machine control complete
* Arcs complete (no helices yet)
* Buffered, non blocking, asynchronous stepping so the rest of the system is free to generate new steps and parse 
  g-code while the steppers are still steppin' 
* Basic serial protocol complete
* Runs on real hardware, pulses verified on scope. Not tested on actual stepper motors. Still waiting for my micRo kit from Lumenlab.com

Limitations:
* Limited GCode-support. Focus on the kind of GCode produced by CAM tools. Leave human GCoders frustrated.
* No rotation axes, only x, y and z.



