# EmlidIMU
Configuration and Programs for activating IMU data on Emlid Reach

An attempt to document how I as able to get IMU data functioning on the Emild Reach.

See PDF file for a first start at line by line instructions.
https://github.com/87yj/EmlidIMU/blob/master/Reach%20IMU%20setup.pdf

Sources (beyond the threads in the emlid board) deserving credit:
https://github.com/RTIMULib/RTIMULib2/tree/master/Linux
http://kingtidesailing.blogspot.com/2016/02/how-to-setup-mpu-9250-on-raspberry-pi_25.html

The Create_PAOGI.py script can be used to read Emlid GPS and IMU data and create a proprietary $PAOGI NMEA sentence that can be sent to AgOpenGPS software.

Create_PAOGI.py is a python script that will take the GPS and IMU output from an Emlid Reach and build a combined $PAOGI sentence that can be read into AgOpenGPS.  Requires RTIMULib to be configured on the Emlid and script must be run from the same director where the RTIMULib.ini file is located.

Keyboard inputs can be entered as the program is running to flip the state of these items:
"gga" = Toggles GGA sentence on and off in the message sent out to the UDP port
"rmc" = Toggles RMC sentence on and off in the message sent out to the UDP port
"vpar=x.xxx"  ex: vapr = 0.001 : Sets the Kalman process filter to the value entered x.xxx
"s" : Turns file save on and off.  note: directory named datafile must be present the next level down from where the program is run
"m" : Toggles the display of output messages printed to the screen (SSH)
"debug"  : Toggles showing the raw roll and Kalman filteres roll on the screen
