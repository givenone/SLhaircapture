cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(OpenCV_DIR /usr/local/lib/cmake/opencv4)

project( DisplayImage )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( temp stdafx.cpp cvScanProCam.cpp cvStructuredLight.cpp )
target_link_libraries( temp ${OpenCV_LIBS} )