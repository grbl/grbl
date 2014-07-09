#Grbl
------------

This branch serves only as a developmental platform for working on new ideas that may eventually be installed into Grbl's edge branch. Please do not use as there is no guarantee this code base is up-to-date or working.

------------

Grbl is a no-compromise, high performance, low cost alternative to parallel-port-based motion control for CNC milling. It will run on a vanilla Arduino (Duemillanove/Uno) as long as it sports an Atmega 328. (Other AVR CPUs are unofficially supported as well.)

The controller is written in highly optimized C utilizing every clever feature of the AVR-chips to achieve precise timing and asynchronous operation. It is able to maintain up to 30kHz of stable, jitter free control pulses.

It accepts standards-compliant G-code and has been tested with the output of several CAM tools with no problems. Arcs, circles and helical motion are fully supported, as well as, other basic functional g-code commands. Although canned cycles, functions, and variables are not currently supported (may in the future), GUIs can be built or modified easily to translate to straight g-code for Grbl.

Grbl includes full acceleration management with look ahead. That means the controller will look up to 18 motions into the future and plan its velocities ahead to deliver smooth acceleration and jerk-free cornering.

##Changelog for v0.9 from v0.8
  - **ALPHA status: Under heavy development.**
  - New stepper algorithm:  Complete overhaul of the handling of the stepper driver to simplify and reduce task time per ISR tick. Much smoother operation with the new Adaptive Multi-Axis Step Smoothing (AMASS) algorithm which does what its name implies. Users should audibly hear significant differences in how their machines move and see improved overall performance!
  - Planner optimizations: Planning computations improved four-fold or more by optimizing end-to-end operations. Changes include streaming optimizations by ignoring already optimized blocks and removing redundant variables and computations and offloading them to the stepper algorithm to compute on an ad-hoc basis.
  - Planner stability: Previous Grbl planners have all had a corruption issue in rare circumstances that becomes particularly problematic at high step frequencies and when jogging. The new planner is robust and incorruptible, meaning that we can fearlessly drive Grbl to it's highest limits. Combined with the new stepper algorithm and planner optimizations, this means 5x to 10x performance increases in our testing! This is all achieved through the introduction of an intermediary step segment buffer that "checks-out" steps from the planner buffer in real-time. 
  - Acceleration independence: Each axes may be defined with different acceleration parameters and Grbl will automagically calculate the maximum acceleration through a path depending on the direction traveled. This is very useful for machine that have very different axes properties, like the ShapeOko z-axis.
  - Maximum velocity independence: As with acceleration, the maximum velocity of individual axes may be defined. All seek/rapids motions will move at these maximum rates, but never exceed any one axes. So, when two or more axes move, the limiting axis will move at its maximum rate, while the other axes are scaled down.
  - Significantly improved arc performance: Arcs are now defined in terms of chordal tolerance, rather than segment length. Chordal tolerance will automatically scale all arc line segments depending on arc radius, such that the error does not exceed the tolerance value (default: 0.002 mm.) So, for larger radii arcs, Grbl can move faster through them, because the segments are always longer and the planner has more distance to plan with.
  - Soft limits: Checks if any motion command exceeds workspace limits. Alarms out when found. Another safety feature, but, unlike hard limits, position does not get lost, as it forces a feed hold before erroring out.
  - CPU pin mapping: In an effort for Grbl to be compatible with other AVR architectures, such as the 1280 or 2560, a new cpu_map.h pin configuration file has been created to allow Grbl to be compiled for them. This is currently user supported, so your mileage may vary. If you run across a bug, please let us know or better send us a fix! Thanks in advance!
  - New Grbl SIMULATOR by @jgeisler: A completely independent wrapper of the Grbl main source code that may be compiled as an executable on a computer. No Arduino required. Simply simulates the responses of Grbl as if it was on an Arduino. May be used for many things: checking out how Grbl works, pre-process moves for GUI graphics, debugging of new features, etc. Much left to do, but potentially very powerful, as the dummy AVR variables can be written to output anything you need. 
  - Homing routine updated: Sets workspace volume in all negative space regardless of limit switch position. Common on pro CNCs. Now tied directly into the main planner and stepper modules to reduce flash space and allow maximum speeds during seeking.
  - Combined limit pins capability: Limit switches can be combined to share the same pins to free up precious I/O pins for other purposes. When combined, users must adjust the homing cycle mask in config.h to not home the axes on a shared pin at the same time. 
  - Variable spindle speed output: Available only as a compile-time option through the config.h file. Enables PWM output for 'S' g-code commands. Enabling this feature will swap the Z-limit D11 pin and spindle enable D12 pin to access the hardware PWM on pin D12. The Z-limit pin, now on D12, should work just as it did before.
  - Increased serial baud rate: Default serial baudrate is now 115200, because the new planner and stepper allows much higher processing and execution speeds that 9600 baud was just not cutting it.
  - Feedrate overrides: (Slated for v1.0 release) The framework to enable feedrate overrides is in-place with v0.9, but the minor details has not yet been installed.
  - Jogging controls: (Slated for v1.0 release) Methodology needs to be to figured out first. Could be dropped due to flash space concerns.
  
_The project was initially inspired by the Arduino GCode Interpreter by Mike Ellery_
