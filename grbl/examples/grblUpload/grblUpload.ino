/***********************************************************************
This sketch compiles and uploads Grbl to your 328p-based Arduino! 

To use:
- First make sure you have imported Grbl source code into your Arduino
  IDE. There are details on our Github website on how to do this.

- Select your Arduino Board and Serial Port in the Tools drop-down menu.
  NOTE: Grbl only officially supports 328p-based Arduinos, like the Uno.
  Using other boards will likely not work!

- Then just click 'Upload'. That's it!

For advanced users:
  If you'd like to see what else Grbl can do, there are some additional
  options for customization and features you can enable or disable. 
  Navigate your file system to where the Arduino IDE has stored the Grbl 
  source code files, open the 'config.h' file in your favorite text 
  editor. Inside are dozens of feature descriptions and #defines. Simply
  comment or uncomment the #defines or alter their assigned values, save
  your changes, and then click 'Upload' here. 

Copyright (c) 2015 Sungeun K. Jeon
Released under the MIT-license. See license.txt for details.
***********************************************************************/

#include <grbl.h>

// Do not alter this file!
