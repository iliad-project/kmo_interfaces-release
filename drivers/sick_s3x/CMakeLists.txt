cmake_minimum_required(VERSION 2.8.3)
project(sick_s3x)

find_package(catkin REQUIRED
  roscpp
  tf 
)
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES sick_s3x
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(sick_s3x_node src/serialcomm_s300.cpp src/sicks300.cpp)
target_link_libraries(sick_s3x_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS sick_s3x_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
