![GitHub Logo](/doc/media/Grbl Logo 250px.png)


***

_**This is the development branch for Grbl v1.0 for the Arduino Mega2560 only. All Arduino Uno(328p) references have been stripped and will not work. The intent is to expand Grbl's functionalities without the flash and memory constraints of the smaller 328p. Most changes here will be mirrored onto the main Grbl repo when it makes sense to, and vice versa.**_

_**Please keep in mind, this version of Grbl is currently beta, so use with caution. If you'd like to help, please report any bugs or oddities that you find! Thanks!**_

***

Grbl is a no-compromise, high performance, low cost alternative to parallel-port-based motion control for CNC milling. This version of Grbl runs on an Arduino Mega2560 only.

The controller is written in highly optimized C utilizing every clever feature of the AVR-chips to achieve precise timing and asynchronous operation. It is able to maintain up to 30kHz of stable, jitter free control pulses.

It accepts standards-compliant g-code and has been tested with the output of several CAM tools with no problems. Arcs, circles and helical motion are fully supported, as well as, all other primary g-code commands. Macro functions, variables, and most canned cycles are not supported, but we think GUIs can do a much better job at translating them into straight g-code anyhow.

Grbl includes full acceleration management with look ahead. That means the controller will look up to 24 motions into the future and plan its velocities ahead to deliver smooth acceleration and jerk-free cornering.

* [Licensing](https://github.com/grbl/grbl/wiki/Licensing): Grbl is free software, released under the GPLv3 license.

* For more information and help, check out our **[Wiki pages!](https://github.com/grbl/grbl/wiki)** If you find that the information is out-dated, please to help us keep it updated by editing it or notifying our community! Thanks!

* Lead Developer [_2011 - Current_]: Sungeun(Sonny) K. Jeon, Ph.D. (USA) aka @chamnit

* Lead Developer [_2009 - 2011_]: Simen Svale Skogsrud (Norway)

***

### Official Supporters of the Grbl CNC Project
![Official Supporters](https://dl.dropboxusercontent.com/u/2221997/Contributors.png)


***

##Update Summary for v1.0c
- **IMPORTANT:** Your EEPROM will be wiped and restored with new settings. This is due to the addition of two new spindle speed '$' settings.

- Altered limit pin status reports from `Lim:000` to `Pin:000|0|0000`, where the `|` delimiters separate the new probe state and control pin states. Each new field may be disabled by the `$10` Grbl setting. NOTE: Commenting `REPORT_ALL_PIN_STATES` in config.h reverts to old `Lim:` reports, if needed.

- New safety door parking motion as a compile-option. Grbl will retract, disable the spindle/coolant, and park near Z max. When resumed, it will perform these task in reverse order and continue the program. Highly configurable. See config.h for details.

- New '$' Grbl settings for max and min spindle rpm. Allows for tweaking the PWM output to more closely match true spindle rpm. When max rpm is set to zero or less than min rpm, the PWM pin D11 will act like a simple enable on/off output.

- Updated G28 and G30 behavior from NIST to LinuxCNC g-code description. In short, if a intermediate motion is specified, only the axes specified will move to the stored coordinates, not all axes as before.

- A few bug fixes and lots of refactoring to make the code more efficient and flexible.


-

``` 
List of Supported G-Codes in Grbl v0.9 Master:
  - Non-Modal Commands: G4, G10L2, G10L20, G28, G30, G28.1, G30.1, G53, G92, G92.1
  - Motion Modes: G0, G1, G2, G3, G38.2, G38.3, G38.4, G38.5, G80
  - Feed Rate Modes: G93, G94
  - Unit Modes: G20, G21
  - Distance Modes: G90, G91
  - Arc IJK Distance Modes: G91.1
  - Plane Select Modes: G17, G18, G19
  - Tool Length Offset Modes: G43.1, G49
  - Cutter Compensation Modes: G40
  - Coordinate System Modes: G54, G55, G56, G57, G58, G59
  - Control Modes: G61
  - Program Flow: M0, M1, M2, M30*
  - Coolant Control: M7*, M8, M9
  - Spindle Control: M3, M4, M5
  - Valid Non-Command Words: F, I, J, K, L, N, P, R, S, T, X, Y, Z
```
