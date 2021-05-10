cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME tod_helper_functions)
project(${PROJECT_NAME})

set(DEPRECATION_FLAG "-Wno-deprecated-declarations -Wno-format")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEPRECATION_FLAG}")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
    roscpp tf2 sensor_msgs tod_msgs)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp tf2 sensor_msgs tod_msgs
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
