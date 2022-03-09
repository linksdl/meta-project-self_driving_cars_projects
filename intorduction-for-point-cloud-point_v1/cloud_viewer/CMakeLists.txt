project(cloud_viewer)

add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
        cloud_viewer.cpp
        )
find_package(PCL REQUIRED)
#include
target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${PCL_INCLUDE_DIRS} )
#link
target_link_directories(${PROJECT_NAME}
    PUBLIC
        ${PCL_LIBRARY_DIRS}
        )
target_link_libraries(${PROJECT_NAME}
        ${PCL_LIBRARIES}
        )

