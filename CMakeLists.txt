cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(main)

find_package(PCL 1.2 REQUIRED)
if(PCL_FOUND)
    message("PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
    message("PCL_LIBRARY_DIRS: ${PCL_LIBRARY_DIRS}")
    message("PCL_LIBRARIES: ${PCL_LIBRARIES}")
    message("PCL_COMPONENTS: ${PCL_COMPONENTS}")
    message("PCL_VERSION: ${PCL_VERSION}")
else()
    message("pcl not found")
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(OpenCV_FOUND 1)
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable (main main.cpp src/loader.h src/loader.cpp)
target_link_libraries (main ${PCL_LIBRARIES} ${OpenCV_LIBS})
