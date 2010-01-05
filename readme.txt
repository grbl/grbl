Grbl - An embedded rs274/ngc (g-code) interpreter, CNC controller, readout and exerciser for the AVR series of microcontrollers.
Inspired by the Arduino GCode Interpreter by Mike Ellery

Status:
* Runs on atmega168/arduino.
* GCode interpreter complete
* Linear interpolation machine control complete
* Arcs and helical interpolation complete
* Buffered, non blocking, asynchronous stepping so the rest of the system is free to generate new steps and parse 
  g-code while the steppers are still steppin' 
* Basic serial protocol complete
* Stepper pulses verified on scope and tested with stepper motors, motion and rates verified with simulator,
  but not tested on real CNC-rig. Still waiting for my micRo kit from Lumenlab.com

Pending: 
* Optional support for a alphanumeric LCD readout, a joystick and a few buttons for program control
* Optional support for automated cutter length calibration when milling
* Support "headless" fabrication by buffering all code to SD-card or similar
* Smooth feed rate interpolation

Limitations:
* Limited GCode-support. Focus on the kind of GCode produced by CAM tools. Leave human GCoders frustrated.
* No rotation axes, only x, y and z.
