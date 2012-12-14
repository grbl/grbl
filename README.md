#Grbl - An embedded g-code interpreter and motion-controller for the Arduino/AVR328 microcontroller
------------

Grbl is a no-compromise, high performance, low cost alternative to parallel-port-based motion control for CNC milling. It will run on a vanilla Arduino (Duemillanove/Uno) as long as it sports an Atmega 328. 

The controller is written in highly optimized C utilizing every clever feature of the AVR-chips to achieve precise timing and asynchronous operation. It is able to m	aintain more than 30kHz of stable, jitter free control pulses.

It accepts standards-compliant G-code and has been tested with the output of several CAM tools with no problems. Arcs, circles and helical motion are fully supported, as well as, other basic functional g-code commands. Functions and variables are not currently supported, but may be included in future releases in a form of a pre-processor.

Grbl includes full acceleration management with look ahead. That means the controller will look up to 18 motions into the future and plan its velocities ahead to deliver smooth acceleration and jerk-free cornering.

##Changelog for v0.9 from v0.8
  - **ALPHA status: Under heavy development.**
  - New stepper algorithm:  Based on the Pramod Ranade inverse time algorithm, but modified to ensure steps are executed exactly. This algorithm performs a constant timer tick and has a hard limit of 30kHz maximum step frequency. It is also highly tuneable and should be very easy to port to other microcontroller architectures.
  - Planner optimizations: Multiple changes to increase planner execution speed and removed redundant variables.
  - Acceleration independence: Each axes may be defined with different acceleration parameters and Grbl will automagically calculate the maximum acceleration through a path depending on the direction traveled. This is very useful for machine that have very different axes properties, like the ShapeOko z-axis.
  - Feedrate overrides: In the works, but planner has begun to be re-factored for this feature.
  
_The project was initially inspired by the Arduino GCode Interpreter by Mike Ellery_
