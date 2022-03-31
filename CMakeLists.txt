cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(main)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(OpenCV_FOUND 1)
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable (main main.cpp src/loader.h src/loader.cpp src/PGMbReader.cpp src/PGMbReader.h)
target_link_libraries (main ${PCL_LIBRARIES} ${OpenCV_LIBS})
