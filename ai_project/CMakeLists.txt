cmake_minimum_required(VERSION 2.8.3)
project(ai_project)

execute_process(COMMAND pwd OUTPUT_VARIABLE DIR)

#message("project: ${DIR}")
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

add_executable(app src/main.cpp)

include_directories(./include/)
#string(REPLACE "build" "lib" WORK_DIR ${DIR})

message("${WORK_DIR}")
set(ASTRA_SDK_LIB "/home/renxl/work/astra/astra-dev/ai_project/lib")
#set(ASTRA_SDK_LIB "/home/renxl/work/samba-share/astra/astra-dev/ai_project/lib")

target_link_libraries(app
	dl
	${ASTRA_SDK_LIB}/libastra.so 
	${ASTRA_SDK_LIB}/libastra_core.so
	${ASTRA_SDK_LIB}/libastra_core_api.so
	sfml-graphics
	sfml-system
	sfml-window
	pthread
)






