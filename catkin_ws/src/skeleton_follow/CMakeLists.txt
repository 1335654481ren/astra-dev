cmake_minimum_required(VERSION 2.8.3)
project(skeleton_follow)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

add_executable(skeleton_follow src/main.cpp)
target_link_libraries(skeleton_follow ${catkin_LIBRARIES})
include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(./include/)
include_directories(./common/)

LINK_DIRECTORIES(./lib)

set(ASTRA_SDK_LIB "/home/renxl/work/samba-share/astra/catkin_ws/src/skeleton_follow/lib")

target_link_libraries(skeleton_follow
	dl
	${ASTRA_SDK_LIB}/libastra.so 
	${ASTRA_SDK_LIB}/libastra_core.so
	${ASTRA_SDK_LIB}/libastra_core_api.so
	sfml-graphics
	sfml-system
	sfml-window
	pthread
)



