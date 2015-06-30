# How to get started with the B3R1 project. #
The B3R1 is a self balancing two wheeled robot that uses the AVR Butterfly micro controller and the WinAVR development environment.


## Things you will need to get started: ##
  * AVR Butterfly or with code changes a Similar ATmega.
  * 2-DOF IMU from sparkfun or Similar (Accelerometer/Rate Gyro).
  * WinAVR - I suggest you google "WinAVR" and download the version for your computer
  * AVRStudio - I suggest you google "AVRStudio" and download the version for your computer
  * subversion client. We suggest TortoiseSVN http://tortoisesvn.net/downloads

## TottoiseSVN comes with a good manual but here's a quick start: ##
  * Create a file on your computer. I called mine "B3R1"
  * Right click on the file
  * Select "SVN Checkout"
  * Non-members paste "http://b3r1.googlecode.com/svn/trunk/"

## Adding a Folder to the repository ##
  * This is not needed if you are only adding a file.
  * It is probably best to start with your folders synchronized with the repository.
  * On your computer create the folder
  * Go up one level to the parent folder and right-click.
  * Select TortoiseSVN and select "add"
  * Select TortoiseSVN again and select commit