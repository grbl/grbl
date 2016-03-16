# grbl-polar

***

Grbl v0.9 with 2D polar movement and servo motor support.

This modification has been done to work in our graffiti machine. It is need 2d polar movement to move the spray and a servo motor suport feature to activate the spray.

  define POLAR: changes the input gcode cartesian coordinates to polar machine movement. To active this feature is                      required to know the distance between the two motors and to do the homing cycle when the machine is                     initialized because is needed to know the idle position at anytime.
  
  define RC_SERVO: Use the PIN D11 to drive the servo. Use the commands M03 Sxxx (xxx between 0 and 255) to rotate the                     servo between 0-180. The command M05 turn the servo to zero degrees. (https://github.com/robottini/grbl-servo)
  
  GRBL: (https://github.com/grbl/grbl)
