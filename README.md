![GitHub Logo](https://github.com/gnea/gnea-Media/blob/master/Grbl%20Logo/Grbl%20Logo%20250px.png?raw=true)
***

### Grbl v1.1 has been released [here](https://github.com/gnea/grbl/releases)!
### Notice: This site will be phased out and moved to the new one!

***

Grbl is a no-compromise, high performance, low cost alternative to parallel-port-based motion control for CNC milling. It will run on a vanilla Arduino (Duemillanove/Uno) as long as it sports an Atmega 328. 

The controller is written in highly optimized C utilizing every clever feature of the AVR-chips to achieve precise timing and asynchronous operation. It is able to maintain up to 30kHz of stable, jitter free control pulses.

It accepts standards-compliant g-code and has been tested with the output of several CAM tools with no problems. Arcs, circles and helical motion are fully supported, as well as, all other primary g-code commands. Macro functions, variables, and most canned cycles are not supported, but we think GUIs can do a much better job at translating them into straight g-code anyhow.

Grbl includes full acceleration management with look ahead. That means the controller will look up to 18 motions into the future and plan its velocities ahead to deliver smooth acceleration and jerk-free cornering.

* [Licensing](https://github.com/grbl/grbl/wiki/Licensing): Grbl is free software, released under the GPLv3 license.

* For more information and help, check out our **[Wiki pages!](https://github.com/grbl/grbl/wiki)** If you find that the information is out-dated, please to help us keep it updated by editing it or notifying our community! Thanks!

* Lead Developer [_2011 - Current_]: Sungeun(Sonny) K. Jeon, Ph.D. (USA) aka @chamnit

* Lead Developer [_2009 - 2011_]: Simen Svale Skogsrud (Norway). aka The Originator/Creator/Pioneer/Father of Grbl.

***

### Official Supporters of the Grbl CNC Project
![Official Supporters](https://github.com/gnea/gnea-Media/blob/master/Contributors.png?raw=true)

***

_**Master Branch:**_
* [Grbl v0.9j Atmega328p 16mhz 115200baud with generic defaults](http://bit.ly/1I8Ey4S) _(2016-03-17)_
  - **IMPORTANT INFO WHEN UPGRADING TO GRBL v0.9 :** 
  - Baudrate is now **115200** (Up from 9600). 
  - Homing cycle updated. Located based on switch trigger, rather than release point.
  - Variable spindle is now enabled by default. Z-limit(D12) and spindle enable(D11) have switched to access the hardware PWM on D11. Homing will not work if you do not re-wire your Z-limit switch to D12.

_**Archives:**_
* [Grbl v0.9i Atmega328p 16mhz 115200baud with generic defaults](http://bit.ly/1EiviDk) 
* [Grbl v0.9g Atmega328p 16mhz 115200baud with generic defaults](http://bit.ly/1m8E1Qa) 
* [Grbl v0.8c Atmega328p 16mhz 9600baud](http://bit.ly/SSdCJE)
* [Grbl v0.7d Atmega328p 16mhz 9600baud](http://bit.ly/ZhL15G)
* [Grbl v0.6b Atmega328p 16mhz 9600baud](http://bit.ly/VD04A5)
* [Grbl v0.51 Atmega328p 16mhz 9600baud](http://bit.ly/W75BS1)
* [Grbl v0.6b Atmega168 16mhz 9600baud](http://bit.ly/SScWnE)
* [Grbl v0.51 Atmega168 16mhz 9600baud](http://bit.ly/VXyrYu)


***

## Update Summary for v0.9j
  - **Restore EEPROM feature:** A new set of restore EEPROM features to help OEMs and users reset their Grbl installation to the build defaults. See Configuring Grbl Wiki for details.
  - **More configuration options for input pins**
  - **Bug fixes including:** Soft limit error handling, disable spindle when S0, g-code reporting of G38.x.
  
## Update Summary for v0.9i
  - **IMPORTANT:**
    - **Homing cycle updated. Locates based on trigger point, rather than release point.**
    - **System tweaks: $14 cycle auto-start has been removed. No more QUEUE state.**
  - **New G-Codes** 
  - **CoreXY Support**
  - **Safety Door Support**
  - **Full Limit and Control Pin Configurability**
  - **Additional Compile-Time Feature Options**

## Update Summary for v0.9h from v0.8
  - **IMPORTANT:**
    - **Default serial baudrate is now 115200! (Up from 9600)**
    - **Z-limit(D12) and spindle enable(D11) pins have switched to support variable spindle!**
  - **Super Smooth Stepper Algorithm**
  - **Stability and Robustness Updates**
  - **(x4)+ Faster Planner**
  - **Compile-able via Arduino IDE!**
  - **G-Code Parser Overhaul**
  - **Independent Acceleration and Velocity Settings**
  - **Soft Limits**
  - **Probing**
  - **Dynamic Tool Length Offsets**
  - **Improved Arc Performance**
  - **CPU Pin Mapping**
  - **New Grbl SIMULATOR! (by @jgeisler and @ashelly)**
  - **Configurable Real-time Status Reporting**
  - **Updated Homing Routine**
  - **Optional Limit Pin Sharing**
  - **Optional Variable Spindle Speed Output**
  - **Additional Compile-Time Feature Options**

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

-------------
Grbl is an open-source project and fueled by the free-time of our intrepid administrators and altruistic users. If you'd like to donate, all proceeds will be used to help fund supporting hardware and testing equipment. Thank you!

[![Donate](https://www.paypalobjects.com/en_US/i/btn/btn_donate_LG.gif)](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=CUGXJHXA36BYW)
