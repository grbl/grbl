# grbl-polar

***
Graffiti robot firmware base on [Grbl v0.9](https://github.com/grbl/grbl)

The implemented kinematics allow a 2 string + gravity system (as in [hektor](http://juerglehni.com/works/hektor)), and the pwm support allows triggering the spray using a servo-motor.

additional features:
  * define POLAR: swaps from cartesian to polar kinematics. It's required to set up the distance between the motors. Homing at startup is essential, otherwisse positioning can not be achieved.
  * define RC_SERVO: Use PIN D11 to drive the servo. Use the commands M03 Sxxx (xxx between 0 and 255) to rotate the servo between 0-180. The command M05 turns the servo to zero degrees. [source](https://github.com/robottini/grbl-servo)
  * 
  
![alt text](https://github.com/ilaro-org/grbl-polar/blob/master/v0.jpg "first test at hangar.org")



GPLv3 license

