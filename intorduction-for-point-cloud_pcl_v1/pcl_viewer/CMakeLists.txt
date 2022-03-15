cmake_minimum_required(VERSION 3.21)
project(pcl_viewer)

set(CMAKE_CXX_STANDARD 11)

# set (PCL_DIR "your path")
find_package(PCL 1.12.0 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCL})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

set(SOURCE_FILES main.cpp)
add_executable(pcl_viewer main.cpp)

target_link_libraries(pcl_viewer ${PCL_LIBRARIES})
