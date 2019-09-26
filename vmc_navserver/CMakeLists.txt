cmake_minimum_required(VERSION 2.8.3)
project(vmc_navserver)

find_package(catkin REQUIRED COMPONENTS 
  cmake_modules
  message_generation 
  std_msgs 
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  VMCEncodersStamped.msg  
  VMCLaserWay.msg  
  VMCLaserWayStamped.msg  
  VMCSingleEncoder.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
)

###########
## Build ##
###########

include_directories(
)
