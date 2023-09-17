cmake_minimum_required(VERSION 3.0.2)
project(my_rb1_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(
  FILES
  Rotate.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
catkin_package(
	CATKIN_DEPENDS
	roscpp
	std_msgs
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rotate_service src/rotate_service.cpp)

add_dependencies(rotate_service ${rotate_service_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(rotate_service ${catkin_LIBRARIES})
