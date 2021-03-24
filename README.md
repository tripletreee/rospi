# rospi
1. Install ROS on RPi 3B Raspbian Jessie:
http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi
Notes:
	1. Install the recommended ROS-Comm version
	2. Before building the catkin workspace, check if the swap space is big enough (1 GB). The swap space can be changed by editing the dphys-swapfile in etc/dphys-swapfile and setting CONF_SWAPSIZE = 1024.

2. Communication between RPi 3B and PC over LAN:
https://razbotics.wordpress.com/2018/01/23/ros-distributed-systems/

3. Install ROS driver for Pico Flexx
https://github.com/code-iai/pico_flexx_driver
Notes: There are extra packages required for the driver. Since thr RPi 3B ROS was installed from source, so the installation for new packages should follow Section 4.2 in 
http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi

# source code
The source code contains two parts, laptop workspace and RPi workspace.

ROS packages in laptop:
1. communication: A basic publisher & subscriber node to test the communication between laptop and RPi when running a common ros core.

2. controller: Move the car by hitting up, down, left, and right buttons on the keyboard.

ROS packages in RPi:
1. communication_test: A basic publisher & subscriber node to test the communication between laptop and RPi when running a common ros core.

2. pico_flexx_driver

3. motor: motor communication interface to keyboard control

4. imu_driver: ROS driver for IMU. Create ROS interface on top of the original driver:
https://github.com/DavidEGrayson/minimu9-ahrs


# how to run
1. In RPi, cd to the ~/catkin_ws, then run
   roslaunch src/car.launch
This will start the pico_flexx camera and the keyboard control.

2. In laptop, in the workspace, run
   roslaunch src/keyboard_ctrl.launch
Then the car can be controlled by keyboard


Project architecture
![alt text](/Diagrams/Architecture.jpg)

On Raspberry Pi, use I2C to set speed.
Install SMBUS python package on RPI:
https://github.com/kplindegaard/smbus2





