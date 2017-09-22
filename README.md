# kmo_interfaces

This stack contains interface modules to communicate with the onboard controller (CVC600/VMC500) using sockets. It also contains the drivers needed to steer the vehicles (snowwhites/cititrucks) via CAN and drivers for sensors that are commonly mounted (for now a SICK S300).

This package depends on the navigation_oru stack.


## Installation instructions

The installation assumes you have Ubuntu 16.04 LTS [ROS Kinetic] or Ubntu 14.04 LTS [ROS Indigo] and that you have installed the [navigation_oru](https://github.com/OrebroUniversity/navigation_oru-release) stack.

#### Install the leaf CAN driver

The source code of this driver (linuxcan.tar.gz) is available in the external_libs folder.

`$ tar xvfz linuxcan.tar.gz`

`$ cd linuxcan`

`$ make`

`$ sudo make install`


#### Install the ROS packages

Place the sources in your catkin workspace and run

`$ caktin_make`

#### Running the real truck

Note that this assumes that the truck is located in the basement @ ORU.

`$ roslaunch kmo_navserver real_single_truck.launch`

