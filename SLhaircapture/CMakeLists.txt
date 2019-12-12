cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( scanner stdafx.cpp cvScanProCam.cpp cvStructuredLight.cpp cvCalibrateProCam.cpp cvUtilProCam.cpp)
target_link_libraries( temp ${OpenCV_LIBS} )
