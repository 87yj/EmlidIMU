# EmlidIMU
Configuration and Programs for activating IMU data on Emlid Reach


SSH into the reach ( I use PuTTY from Windows )
Login as:root
root@xxx.xxx.xxx.xxx’s password: emlidreach
cd /
I choose to make a new directory for the files:
root@reach:/#mkdir imu
root@reach:/imu#cd imu
Install the cmake app
root@reach:/imu#opkg install cmake
Now clone the files from git:
root@reach:/imu#git clone https://github.com/richards-tech/RTIMULib2.git
root@reach:/imu#cd RTIMULib2/Linux
root@reach:/imu/RTIMULib2/Linux#vim cMakelists.txt
Edit the list to not build GL
(use arrows to move curor, then press ‘i’ to allow  insert edit mode)
OPTION(BUILD_GL “Build RTIMULibGL” OFF)
Press escape 
:wq to write end quit program
root@reach:/imu/RTIMULib2/Linux#mkdir build
root@reach:/imu/RTIMULib2/Linux#cd build
root@reach:/imu/RTIMULib2/Linux/build#cmake ..
								  #make -j4
								  #make install
I’m confident in the process to this point, after this, what I’m not sure about is the order and which particular directories that you need to go into...so I just start going through the directories and issue the make -j4 and make install command in each! Below is my current guess at the sequence
cd ./RTIMULibDrive
make -j4
Make install
cd ./RTIMULib 
make -j4
		make install
cd ..  (.../Linux#)
	make install
cd RTIMULibCal
make -j4
		make install
I don’t know the exact order, but I know I’m not there yet when I get this message trying to run RTIMULibCal:
RTIMULibCal: error while loading shared libraries: libRTIMULib.so.8: cannot open shared object file: No such file or directory
I know RTIMULib, RTIMULibCAl, RTIMULibDrive are important.
The goal is to get it installed such that you can go into the RTIMULibCal directory and run RTIMULibCal.  When you run the program:
	...RTIMULibCal#RTIMULibCal
When functioning correctly, the first time it will give a message saying Failed to open SPI bus 0…
But is generates an RTIMULib.ini file that you need to edit to set the correct SPI informatiom:
#vim RTIMULib.ini
Edit the following in RTIMULib.ini: (Change the defaults to what is listed in BOLD)
IMUType=7
BusISI2C = false
SPIBus=5
SPISelect=1
I’d recommend sticking with the defaults for everything else at this point.
In the python directory
.../python#python setup.py install
Then
	.../python#cd tests
Next you need to copy the calibrated RTIMULib.ini file into the directory you will be running the application from 
	.../tests#cp ././RTIMULibCal/RTIMULib.ini ./
If all worked well, you can now run the Fusion.py example and get calibrated data out!
	.../tests#python Fusion.py
If you get all zeros, check the RTIMULib.ini file to make sure it’s got the SPI bus config and some numbers listed for the cal data.
If all works, you’ll start seeing a datastream like this:

Sources (beyond the threads in this board) deserving credit:
https://github.com/RTIMULib/RTIMULib2/tree/master/Linux
http://kingtidesailing.blogspot.com/2016/02/how-to-setup-mpu-9250-on-raspberry-pi_25.html
