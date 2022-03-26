cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(main)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (main main.cpp src/loader.h src/loader.cpp src/MyCloud.cpp src/MyCloud.h src/CommandLineParser.cpp src/CommandLineParser.h src/PGMbReader.cpp src/PGMbReader.h)
target_link_libraries (main ${PCL_LIBRARIES})
