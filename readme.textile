h1. Grbl - An embedded g-code interpreter and motion-controller for the Arduino/AVR328 microcontroller
    
Grbl is a no-compromise, high performance, low cost alternative to parallel-port-based motion control for CNC milling. It will run on a vanilla Arduino (Duemillanove/Uno) as long as it sports an Atmega 328. 

The controller is written in highly optimized C utilizing every clever feature of the AVR-chips to achieve precise timing and asynchronous operation. It is able to maintain more than 30kHz of stable, jitter free control pulses.

It accepts standards-compliant G-code and has been tested with the output of several CAM tools with no problems. Arcs, circles and helical motion are fully supported - but no support for tool offsets, functions or variables as these are apocryphal and fell into disuse after humans left G-code authoring to machines some time in the 80s.

Grbl includes full acceleration management with look ahead. That means the controller will look up to 20 motions into the future and plan its velocities ahead to deliver smooth acceleration and jerk-free cornering.

*Important note for Atmega 168 users:* Grbl used to be compatible with both the older Ardunios running atmega 168 and the newer with 328p. This had to go, as I was unable to fit the acceleration management into the 16k code space of the 168. If you want to run Grbl on an 168 I am still maintaining Grbl 0.51 "in the branch called 'v0_51'":https://github.com/simen/grbl/tree/v0_51.

*Note for users upgrading from 0.51 to 0.6:* The new version has new and improved default pin-out. If nothing works when you upgrade, that is because the pulse trains are coming from the wrong pins. This is a simple matter of editing config.h – the whole legacy pin assignment is there for you to uncomment.

_The project was initially inspired by the Arduino GCode Interpreter by Mike Ellery_
