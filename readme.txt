Grbl - An embedded rs274/ngc (g-code) interpreter, cartesian bot controller, readout and exerciser
Inspired by the Arduino GCode Interpreter by Mike Ellery

Status:
* Linear interpolation machine control complete (No arcs yet)
* GCode interpreter complete
* Basic serial protocol complete
* As of yet completely untested in a real environemnt. Still waiting for my micRo kit from Lumenlab.com

Goals: 
* Suitable for both milling and deposition fabrication
* Support GCode from common free and cheap CAM-packages right out of the box
* Optional support for a alphanumeric LCD readout, a joystick and a few buttons for program control
* Optional support for automated cutter length calibration when milling
* Support "headless" fabrication by buffering all code to SD-card or similar

Limitations:
* No support for Arduino software (but will run fine on an Arduino board if you program it with an ISP)
* Limited GCode-support. Focus on the kind of GCode produced by automated CAM tools. Leave human GCoders frustrated.


