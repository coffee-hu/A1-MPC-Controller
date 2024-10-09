cmake_minimum_required(VERSION 3.0.2)
project(pub_joy_node)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pub_joy_node
   CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(pub_joy_node src/qr_telekeyboard.cpp)
target_link_libraries(pub_joy_node ${catkin_LIBRARIES})
