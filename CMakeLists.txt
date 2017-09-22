cmake_minimum_required(VERSION 2.8.3)
project(kmo_navserver)

find_package(catkin REQUIRED COMPONENTS 
  cmake_modules
  message_generation 
  std_msgs 
  roscpp 
  rosconsole 
  rosbag 
  tf 
  tf_conversions
)

find_package(Boost REQUIRED COMPONENTS system program_options)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

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
  INCLUDE_DIRS include
  LIBRARIES kmo_navserver
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

## Declare a cpp library
add_library(kmo_navserver
  src/kmo_navserver.cc
  src/XSocket.cc
  src/kmo_navserver_driver.cc
)
add_dependencies(kmo_navserver kmo_navserver_generate_messages_cpp)

### ---------------------
add_executable(navserver_standalone src/navserver_standalone_main.cc)
add_dependencies(navserver_standalone kmo_navserver_generate_messages_cpp)
target_link_libraries(navserver_standalone kmo_navserver ${catkin_LIBRARIES} -lboost_program_options
)

### ---------------------
add_executable(kmo_navserver_node src/kmo_navserver_node.cc)
add_dependencies(kmo_navserver_node kmo_navserver_generate_messages_cpp)
target_link_libraries(kmo_navserver_node kmo_navserver ${catkin_LIBRARIES} -lboost_program_options)

### ----------------------
add_executable(set_state src/set_state_main.cc)
add_dependencies(set_state kmo_navserver_generate_messages_cpp)
target_link_libraries(set_state kmo_navserver ${catkin_LIBRARIES} -lboost_program_options)

### ----------------------
add_executable(logfile_parser src/logfile_parser.cpp)
add_dependencies(logfile_parser kmo_navserver_generate_messages_cpp)
target_link_libraries(logfile_parser kmo_navserver ${catkin_LIBRARIES}  ${Eigen_LIBRARIES} -lboost_program_options)

#############
## Install ##
#############

install(TARGETS kmo_navserver kmo_navserver_node set_state logfile_parser
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY
   launch
   config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
