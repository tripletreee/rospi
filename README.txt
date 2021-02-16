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
