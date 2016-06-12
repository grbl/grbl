#GRBL 28byj-48 + Servo Motor

This GRBL uses an ugly hack to control two motors unipolar steps as 28byj-48 and also supports a servo motor on pin 11

The motors (28byj-48) are connected to a controller card that uses the chip ULN2003. This board is connected to pins A0, A1, A2, A3 for the Y axis and 2,3,4,5 digital pins to the axis X. The servomotor connected to pin 11 h.

Credits to robottini who did support the servo motor ... link to git https://github.com/robottini/grbl-servo

I tested the code very well with 328p (Arduino Uno, Duemilanove etv), not with 2560 (Arduino Mega) because I did not do the alterations in the file cpu_map_atmega2560.h



