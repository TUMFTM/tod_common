cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME tod_projection_helper)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules" ${CMAKE_MODULE_PATH})
find_package(catkin REQUIRED COMPONENTS roscpp roslib)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp roslib
    )

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
