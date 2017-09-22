cmake_minimum_required(VERSION 2.8.3)
project(kmo_cpi_interface)

find_package(catkin REQUIRED)

find_package(Boost REQUIRED COMPONENTS program_options)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kmo_cpi_interface
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include)
add_library(kmo_cpi_interface
  src/Socket.cpp src/CPISocket.cpp
)

add_executable(move_forks_test src/move_forks_test.cpp)
target_link_libraries(move_forks_test kmo_cpi_interface -lboost_program_options)

add_executable(get_estop_test src/get_estop_test.cpp)
target_link_libraries(get_estop_test kmo_cpi_interface -lboost_program_options)

#############
## Install ##
#############

install(TARGETS move_forks_test get_estop_test kmo_cpi_interface
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       )
