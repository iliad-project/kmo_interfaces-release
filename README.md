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

To check that the driver is properly installed plug in the Kvaser CAN USB adapter and run

`dmesg | grep leaf`

if you get something like

`[19.4232] usbcore: registered new interface driver leaf`

then you are all god to go. A note is that this driver might be needed to be re-built in case you update your kernel. If you do have troubles accessing the CAN dongle after a kernel update, wipe the whole source tree and start all over again, that should sort it out.

#### Install the ROS packages

Place the sources in your catkin workspace and run

`$ caktin_make`

#### Running the real truck

In order to run the real truck you need to change one build parameter in the orunav_mpc/CMakeLists.txt file, change the parameter SW_BUILD_SIMULATION to OFF.

`SET(SW_BUILD_SIMULATION OFF CACHE BOOL          "Build controller to run within simulation." FORCE)`


The following example assumes that the truck is located in the basement @ ORU.

`$ roslaunch kmo_navserver real_single_truck.launch`

Another option is to instead start an empty map.

`$ roslaunch kmo_navserver real_single_truck_empty_map.launch`
