cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(pcl_intro)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${EIGEN3_LIBRARY_DIRS})

add_executable(main src/main.cpp)
target_link_libraries(main ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})