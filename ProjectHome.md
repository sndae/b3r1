# The Project: #
This project is a two wheeled balancing robot that uses the AVR Butterfly for it's micro controller and the Sparkfun 2-DOF IMU.  The robot currently is using two R/C servos that have been modified to provide continuous rotation and servo electronics have been removed so that are basically a gear motor.

# Project Status: #
The project is currently on hold as I so stupidly placed the robot on the roof of the car while loading the car and then drove home.  Need less to say the robot could not hang on as was run over by many many cars.

The robot was balancing quite well on its own, especially on carpet or fabric and not too badly on hard surfaces.  I now know why most balancing robots have pneumatic or soft spongy wheels.

I have pictures of the robot on my website at [jamesronald.us/projects.html](http://jamesronald.us/projects.html). I had replaced the 2-5/8" plastic/rubberband tire wheels with 2-3/4" Dave Brown wheel which helped quite a bit. I would have also liked to try mounting the IMU a bit higher on the bot to see if it made any difference.

# Notes: #
If I build [revision 2](https://code.google.com/p/b3r1/source/detail?r=2) I plan to use faster motors as the R/C servos are just barely fast enough.

# Credits: #
This is by no means a unique effort.
  * BBOT - Phil Davis and Brandon Heller
  * ARTIST - Dirk Uffmann

# The Hardware: #
  * ATMEL AVR Butterfly (atMega169)running at 8 MHz.
  * Sparkfun 2-DOF IMU (ADXL203 accelerometer/ADXRS401 Rate Gyro)
  * Two R/C Servos Modified for continuous rotation and servo electronics have been removed so that are basically a gear motor.
  * SN754410 chip as a Dual H-bridge motor driver.
  * 7.4 Volt DIY Li-Ion Battery pack made with 18650 Li-Ion batteries from a failed laptop battery pack.
  * LM2940 Regulated 5 volts for the Microprocessor, IMU and SN754410 V-Logic (pin16). For the V-Motor (pin 8) use unregulated voltage.

# The Code: #
In a nut shell, the Robot uses a Kalman filter to calculate current angle and rate estimates based on inputs from the Accelerometer and Rate Gyro.  Base on the robots known balance angle, it calculates its error and sets the motors in motion to correct the error. Since the robot has no wheel encoders to measure a correction, it just calculates a new pwm speed value base on (current angle **Kp) + (rate** Kd). The code for the Ki term is there, but it's not currently being used.  Currently, this runs in a loop at 50 times per second.

I will add more details regarding the code in the future. http://code.google.com/p/b3r1/source/checkout
