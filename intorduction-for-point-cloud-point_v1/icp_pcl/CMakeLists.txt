cmake_minimum_required(VERSION 3.21)
project(icp_pcl)
add_executable(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.12.0 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(icp_pcl main.cpp)
target_link_libraries(icp_pcl ${PCL_LIBRARIES})