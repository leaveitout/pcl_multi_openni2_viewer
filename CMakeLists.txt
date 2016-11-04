cmake_minimum_required(VERSION 3.3)

project(pcl_multi_openni2_viewer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCE_FILES main.cpp MultiOpenNI2Viewer.h ViewerNode.h Util.h Timer.cpp Timer.h)

add_executable(pcl_multi_openni2_viewer ${SOURCE_FILES})

target_link_libraries(pcl_multi_openni2_viewer ${PCL_LIBRARIES})
