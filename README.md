#Grbl - An embedded g-code interpreter and motion-controller for the Arduino/AVR328 microcontroller

***

Grbl is a no-compromise, high performance, low cost alternative to parallel-port-based motion control for CNC milling. It will run on a vanilla Arduino (Duemillanove/Uno) as long as it sports an Atmega 328. 

The controller is written in highly optimized C utilizing every clever feature of the AVR-chips to achieve precise timing and asynchronous operation. It is able to maintain up to 30kHz of stable, jitter free control pulses.

It accepts standards-compliant g-code and has been tested with the output of several CAM tools with no problems. Arcs, circles and helical motion are fully supported, as well as, all other primary g-code commands. Macro functions, variables, and most canned cycles are not supported, but we think GUIs can do a much better job at translating them into straight g-code anyhow.

Grbl includes full acceleration management with look ahead. That means the controller will look up to 18 motions into the future and plan its velocities ahead to deliver smooth acceleration and jerk-free cornering.

* [Licensing](https://github.com/grbl/grbl/wiki/Licensing): Grbl v0.9 is free software, released under the GPLv3 license. Obsolete versions of Grbl, v0.8 and prior, are released under the permissive MIT-license. This will ensure Grbl will always be an open-source project while making the code permissive for others.

* For more information and help, check out our **[Wiki pages!](https://github.com/grbl/grbl/wiki)** If you find that the information is out-dated, please to help us keep it updated by editing it or notifying our community! Thanks!

* Lead Developer [_2011 - Current_]: Sonny Jeon, Ph.D. (USA)

* Lead Developer [_2009 - 2011_]: Simen Svale Skogsrud (Norway). aka The Originator/Creator/Pioneer/Father of Grbl.

***

_**Master Branch:**_
* [Grbl v0.9g Atmega328p 16mhz 115200baud with generic defaults](http://bit.ly/1m8E1Qa) _(2014-08-22)_
* [Grbl v0.9g Atmega328p 16mhz 115200baud with ShapeOko2 defaults](http://bit.ly/1kOAzig) _(2014-08-22)_
  - **IMPORTANT INFO WHEN UPGRADING TO GRBL v0.9g:** 
  - Baudrate is now **115200** (Up from 9600). 
  - Settings WILL be overwritten. Please make sure you have a backup. Also, settings have been renumbered and some have changed how they work. See our [Configuring v0.9 Wiki page](https://github.com/grbl/grbl/wiki/Configuring-Grbl-v0.9) for details.

_**Archives:**_
* [Grbl v0.8c Atmega328p 16mhz 9600baud](http://bit.ly/SSdCJE)
* [Grbl v0.7d Atmega328p 16mhz 9600baud](http://bit.ly/ZhL15G)
* [Grbl v0.6b Atmega328p 16mhz 9600baud](http://bit.ly/VD04A5)
* [Grbl v0.51 Atmega328p 16mhz 9600baud](http://bit.ly/W75BS1)
* [Grbl v0.6b Atmega168 16mhz 9600baud](http://bit.ly/SScWnE)
* [Grbl v0.51 Atmega168 16mhz 9600baud](http://bit.ly/VXyrYu)

***

##Update Summary for v0.9 from v0.8
  - **IMPORTANT: Default serial baudrate is now 115200! (Up from 9600). And your settings will be over-written! Make sure to have a backup.**
  - **_NEW_ Super Smooth Stepper Algorithm:**  Complete overhaul of the handling of the stepper driver to simplify and reduce task time per ISR tick. Much smoother operation with the new Adaptive Multi-Axis Step Smoothing (AMASS) algorithm which does what its name implies (see stepper.c source for details). Users should immediately see significant improvements in how their machines move and overall performance!
  - **Stability and Robustness Updates:** Grbl's overall stability has been focused on for this version. The planner and step-execution interface has been completely re-written for robustness and incorruptibility by the introduction of an intermediate step segment buffer that "checks-out" steps from the planner buffer in real-time. This means we can now fearlessly drive Grbl to it's highest limits. Combined with the new stepper algorithm and planner optimizations, this translated to **5x to 10x** overall performance increases in our testing! Also, stability and robustness tests have been reported to easily take 1.4 million (yes, **million**) line g-code programs like a champ!
  - **(x4)+ Faster Planner:** Planning computations improved four-fold or more by optimizing end-to-end operations, which included streamlining the computations and introducing a planner pointer to locate un-improvable portions of the buffer and not waste cycles recomputing them.
  - **Compile-able via Arduino IDE!:** Grbl's source code may be now download and altered, and then be compiled and flashed directly through the Arduino IDE, which should work on all platforms. See the Wiki for details on how to do it.
  - **G-Code Parser Overhaul:** Completely re-written from the ground-up for 100%-compliance* to the g-code standard. (* Parts of the NIST standard are a bit out-dated and arbitrary, so we altered some minor things to make more sense. Differences are outlined in the source code.) We also took steps to allow us to break up the g-code parser into distinct separate tasks, which is key for some future development ideas and improvements.
  - **Independent Acceleration and Velocity Settings:** Each axes may be defined with unique acceleration and velocity parameters and Grbl will automagically calculate the maximum acceleration and velocity through a path depending on the direction traveled. This is very useful for machines that have very different axes properties, like the ShapeOko's z-axis.
  - **Soft Limits:** Checks if any motion command exceeds workspace limits before executing it, and alarms out, if detected. Another safety feature, but, unlike hard limits, position does not get lost, as it forces a feed hold before erroring out. NOTE: This still requires limit switches for homing so Grbl knows where the machine origin is, and the new max axis travel settings configured correctly for the machine.
  - **Probing:** The G38.2 straight probe and G43.1/49 tool offset g-code commands are now supported. A simple probe switch must be connected to the Uno analog pin 5 (normally-open to ground). Grbl will report the probe position back to the user when the probing cycle detects a pin state change.
  - **Tool Length Offsets:** Probing doesn't make sense without tool length offsets(TLO), so we added it! The G43.1 dynamic TLO (described by linuxcnc.org) and G49 TLO cancel commands are now supported. G43.1 dynamic TLO works like the normal G43 TLO(NOT SUPPORTED) but requires an additional axis word with the offset value attached. We did this so Grbl does not have to track and maintain a tool offset database in its memory. Perhaps in the future, we will support a tool database, but not for this version.
  - **Improved Arc Performance:** The larger the arc radius, the faster Grbl will trace it! We are now defining arcs in terms of arc chordal tolerance, rather than a fixed segment length. This automatically scales the arc segment length such that maximum radial error of the segment from the true arc is never more than the chordal tolerance value of a super-accurate default of 0.002 mm. 
  - **CPU Pin Mapping:** In an effort for Grbl to be compatible with other AVR architectures, such as the 1280 or 2560, a new cpu_map.h pin configuration file has been created to allow Grbl to be compiled for them. This is currently user supported, so your mileage may vary. If you run across a bug, please let us know or better send us a fix! Thanks in advance!
  - **New Grbl SIMULATOR! (by @jgeisler and @ashelly):** A completely independent wrapper of the Grbl main source code that may be compiled as an executable on a computer. No Arduino required. Simply simulates the responses of Grbl as if it was on an Arduino. May be used for many things: checking out how Grbl works, pre-process moves for GUI graphics, debugging of new features, etc. Much left to do, but potentially very powerful, as the dummy AVR variables can be written to output anything you need. 
  - **Configurable Real-time Status Reporting:** Users can now customize the type of real-time data Grbl reports back when they issue a '?' status report. This includes data such as: machine position, work position, planner buffer usage, serial RX buffer usage.
  - **Updated Homing Routine:** Sets workspace volume in all negative space regardless of limit switch position. Common on pro CNCs. But, the behavior may be changed by a compile-time option though. Now tied directly into the main planner and stepper modules to reduce flash space and allow maximum speeds during seeking.
  - **Optional Limit Pin Sharing:** Limit switches can be combined to share the same pins to free up precious I/O pins for other purposes. When combined, users must adjust the homing cycle mask in config.h to not home the axes on a shared pin at the same time. Don't worry; hard limits and the homing cycle still work just like they did before.
  - **Optional Variable Spindle Speed Output:** Available only as a compile-time option through the config.h file. Enables PWM output for 'S' g-code commands. Enabling this feature will swap the Z-limit D11 pin and spindle enable D12 pin to access the hardware PWM on pin D12. The Z-limit pin, now on D12, should work just as it did before.
  - **Additional Compile-Time Feature Options:** Line number tracking, real-time feed rate reporting.
  - **SLATED FOR v1.0 DEVELOPMENT** Jogging controls and feedrate/spindle/coolant overrides. (In v0.9, the framework for feedrate overrides are in-place, only the minor details to complete it have yet to be installed.)

-
``` 
List of Supported G-Codes in Grbl v0.9
  - Non-Modal Commands: G4, G10 L2, G10 L20, G28, G30, G28.1, G30.1, G53, G92, G92.1
  - Motion Modes: G0, G1, G2, G3, G38.1, G80
  - Feed Rate Modes: G93, G94
  - Unit Modes: G20, G21
  - Distance Modes: G90, G91
  - Plane Select Modes: G17, G18, G19
  - Tool Length Offset Modes: G43.1, G49
  - Coordinate System Modes: G54, G55, G56, G57, G58, G59
  - Program Flow: M0, M1, M2, M30*
  - Coolant Control: M7*, M8, M9
  - Spindle Control: M3, M4, M5
```

-------------
Grbl is an open-source project and fueled by the free-time of our intrepid administrators and altruistic users. If you'd like to donate, all proceeds will be used to help fund supporting hardware and testing equipment. Thank you!

[![Donate](https://www.paypalobjects.com/en_US/i/btn/btn_donate_LG.gif)](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=EBQWAWQAAT878)

