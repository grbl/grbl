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
  - New stepper algorithm:  Based on an inverse time algorithm, but modified to ensure steps are executed exactly. This algorithm performs a constant timer tick and has a hard limit of 30kHz maximum step frequency. It is also highly tuneable and should be very easy to port to other microcontroller architectures. Overall, a much better, smoother stepper algorithm with the capability of very high speeds.
  - Planner optimizations: Planning computations improved four-fold or more. Changes include streaming optimizations by ignoring already optimized blocks and removing redundant variables and computations and offloading them to the stepper algorithm on an ad-hoc basis.
  - Planner stability: Previous Grbl planners have all had a corruption issue in rare circumstances that becomes particularly problematic at high step frequencies and when jogging. The new planner is robust and incorruptible, meaning that we can fearlessly drive Grbl to it's highest limits. Combined with the new stepper algorithm and planner optimizations, this means 5x to 10x performance increases in our testing! This is all achieved through the introduction of an intermediary step segment buffer that "checks-out" steps from the planner buffer in real-time. 
  - Acceleration independence: Each axes may be defined with different acceleration parameters and Grbl will automagically calculate the maximum acceleration through a path depending on the direction traveled. This is very useful for machine that have very different axes properties, like the ShapeOko z-axis.
  - Maximum velocity independence: As with acceleration, the maximum velocity of individual axes may be defined. All seek/rapids motions will move at these maximum rates, but never exceed any one axes. So, when two or more axes move, the limiting axis will move at its maximum rate, while the other axes are scaled down.
  - Significantly improved arc performance: Arcs are now defined in terms of chordal tolerance, rather than segment length. Chordal tolerance will automatically scale all arc line segments depending on arc radius, such that the error does not exceed the tolerance value (default: 0.005 mm.) So, for larger radii arcs, Grbl can move faster through them, because the segments are always longer and the planner has more distance to plan with.
  - Soft limits: Checks if any motion command exceeds workspace limits. Alarms out when found. Another safety feature, but, unlike hard limits, position does not get lost, as it forces a feed hold before erroring out.
  - Pin mapping: In an effort for Grbl to be compatible with other AVR architectures, such as the 1280 or 2560, a new pin_map.h configuration file has been created to allow Grbl to be compiled for them. This is currently user supported, so your mileage may vary. If you run across a bug, please let us know or better send us a fix! Thanks in advance!
  - New Grbl SIMULATOR by @jgeisler: A completely independent wrapper of the Grbl main source code that may be compiled as an executable on a computer. No Arduino required. Simply simulates the responses of Grbl as if it was on an Arduino. May be used for many things: checking out how Grbl works, pre-process moves for GUI graphics, debugging of new features, etc. Much left to do, but potentially very powerful, as the dummy AVR variables can be written to output anything you need. 
  - Homing routine updated: Sets workspace volume in all negative space regardless of limit switch position. Common on pro CNCs. Now tied directly into the main planner and stepper modules to reduce flash space and allow maximum speeds during seeking.
  - Feedrate overrides: In the works, but planner has begun to be re-factored for this feature.
  - Jogging controls: Methodology needs to be to figured out first. Could be dropped due to flash space concerns. Last item on the agenda.
  
_The project was initially inspired by the Arduino GCode Interpreter by Mike Ellery_
