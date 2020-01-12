cmake_minimum_required(VERSION 2.8.3)
project(skeleton_markers)


add_compile_options(-std=c++11 -O3 -w -g2 -fpermissive)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	tf
	geometry_msgs
	std_msgs
	visualization_msgs
	message_generation
  #cv_bridge
  sensor_msgs
  #image_transport
)
	
# Find Orocos KDL
find_package(orocos_kdl REQUIRED)

# Find OpenNI


set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)


find_package(PkgConfig)
pkg_check_modules(OpenNI REQUIRED libopenni)
find_package(OpenCV REQUIRED)

# Find Glut
find_package(GLUT REQUIRED)

# Find OpenGL
find_package(OpenGL REQUIRED) 


link_directories(
  ${catkin_LIBRARY_DIRS}
  ${orocos_kdl_LIBRARY_DIRS}
  ${OpenNI_LIBRARIES_DIRS}
  ${GLUT_LIBRARIES_DIRS}
  ${OPENGL_LIBRARIES_DIRS}
  ${OpenCV_LIBRARIES_DIRS}
)

include_directories(${catkin_INCLUDE_DIRS}
                    ${OpenNI_INCLUDE_DIRS}
                    ${GLUT_INCLUDE_DIRS}
                    ${orocos_kdl_INCLUDE_DIRS}
                    ${OPENGL_INCLUDE_DIR}
                    ${OpenCV_INCLUDE_DIR}
)

include_directories(./include/)
include_directories(./common/)

add_message_files (
  FILES
  Skeleton.msg
)

generate_messages(
  	DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(
  DEPENDS 
  CATKIN_DEPENDS 
	rospy
	roscpp
	message_runtime
	openni_camera
	openni_tracker
	tf
	visualization_msgs
	std_msgs
	geometry_msgs
)

add_executable(skeleton_tracker
  src/main.cpp
  src/KinectController.cpp
  src/KinectDisplay.cpp)

set(ASTRA_SDK_LIB "/home/renxl/work/astra/astra-dev/catkin_ws/src/skeleton_follow/lib")
#set(ASTRA_SDK_LIB "/home/renxl/work/samba-share/astra/astra-dev/catkin_ws/src/skeleton_follow/lib")

target_link_libraries(skeleton_tracker
  ${catkin_LIBRARIES}
  ${OpenNI_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OPENGL_LIBRARIES}  
  ${orocos_kdl_LIBRARIES}
  ${ASTRA_SDK_LIB}/libastra.so 
  ${ASTRA_SDK_LIB}/libastra_core.so
  ${ASTRA_SDK_LIB}/libastra_core_api.so
  ${ASTRA_SDK_LIB}/libxdo.so
  sfml-graphics
  sfml-system
  sfml-window
  pthread
  dl
)

#install(TARGETS skeleton_tracker RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

message("DEBUG variable catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")
message("DEBUG variable OpenNI_INCLUDE_DIRS: ${OpenNI_INCLUDE_DIRS}")
message("DEBUG variable GLUT_INCLUDE_DIRS: ${GLUT_INCLUDE_DIRS}")
message("DEBUG variable orocos_kdl_INCLUDE_DIRS: ${orocos_kdl_INCLUDE_DIRS}")

