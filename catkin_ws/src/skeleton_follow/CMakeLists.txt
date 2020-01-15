cmake_minimum_required(VERSION 2.8.3)
project(skeleton_follow)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -O3 -w -g2 -fpermissive)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp 
	rospy 
	std_msgs 
	genmsg
	tf
	geometry_msgs
	visualization_msgs
	message_generation
	#cv_bridge
	sensor_msgs
	#image_transport
	message_runtime
)

# Find Orocos KDL
find_package(orocos_kdl REQUIRED)
find_package(PkgConfig)
# Find Glut
find_package(GLUT REQUIRED)
# Find OpenGL
find_package(OpenGL REQUIRED) 

#find_package(OpenCV REQUIRED)

## Declare ROS messages and services
add_message_files (
  FILES
  Skeleton.msg
)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(
  	DEPENDENCIES geometry_msgs std_msgs
)

## Declare a catkin package
catkin_package()

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${orocos_kdl_LIBRARY_DIRS}
  ${OpenNI_LIBRARIES_DIRS}
  ${GLUT_LIBRARIES_DIRS}
  ${OPENGL_LIBRARIES_DIRS}
  #${OpenCV_LIBRARIES_DIRS}
  /usr/lib/python2.7
)

include_directories(include 
	${catkin_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${OpenNI_INCLUDE_DIRS}
	${GLUT_INCLUDE_DIRS}
	${orocos_kdl_INCLUDE_DIRS}
	${OPENGL_INCLUDE_DIR}
	#${OpenCV_INCLUDE_DIR}
	/usr/include/python2.7
)

add_executable(skeleton_tracker src/main.cpp)

#set(ASTRA_SDK_LIB "/home/renxl/work/astra/astra-dev/catkin_ws/src/skeleton_follow/lib")
set(ASTRA_SDK_LIB "/home/renxl/work/samba-share/astra/astra-dev/catkin_ws/src/skeleton_follow/lib")

target_link_libraries(skeleton_tracker
	${catkin_LIBRARIES}
	${OpenNI_LIBRARIES}
	${GLUT_LIBRARIES}
	#${OpenCV_LIBRARIES}
	${OPENGL_LIBRARIES}  
	${orocos_kdl_LIBRARIES}
	${ASTRA_SDK_LIB}/libastra.so 
	${ASTRA_SDK_LIB}/libastra_core.so
	${ASTRA_SDK_LIB}/libastra_core_api.so
	sfml-graphics
	sfml-system
	sfml-window
	python2.7
	pthread
	X11
	xdo
	dl
)



