# Set the minimum required version of cmake for a project.
cmake_minimum_required(VERSION 3.1)

# Set Variables
set(CMAKE_CXX_FLAGS "-std=c++0x") # C++11
set(PROJECT_NAME "coordMatcher")

# Project
project(${PROJECT_NAME} CXX)

# Find library package
find_package(OpenCV REQUIRED)
find_package(k4a REQUIRED)
#find_package(k4abt REQUIRED)
#find_package(k4arecord REQUIRED)
# find_package(VTK REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
#include_directories(~/data/Azure-Kinect-Samples/body-tracking-samples/sample_helper_includes/)
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})
# include(${VTK_USE_FILE})

# File
file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*)
file(GLOB HEADER_FILES ${PROJECT_SOURCE_DIR}/include/*)

# The main program
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
add_executable(${PROJECT_NAME} main.cpp ${SRC_FILES} ${HEADER_FILES})

# Link
target_link_libraries( ${PROJECT_NAME} 
	${OpenCV_LIBS} 
	k4a::k4a
#	k4a::k4arecord
	Eigen3::Eigen
	# ${VTK_LIBRARIES}
)
