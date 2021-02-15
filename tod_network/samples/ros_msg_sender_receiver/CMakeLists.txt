cmake_minimum_required(VERSION 2.8.12)

project(tod_network_tester)

list(APPEND CMAKE_PREFIX_PATH "/opt/ros/melodic/")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    )

find_package(todnetworklibrary)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${catkin_INCLUDE_DIRS}
    )

add_executable(Sender "mainSender.cpp" "ImageExposureStatistics.h"  "encoderConfig.h")
add_executable(Receiver "mainReceiver.cpp" "ImageExposureStatistics.h"  "encoderConfig.h")
target_link_libraries(Sender ${catkin_LIBRARIES} todnetworklibrary)
target_link_libraries(Receiver ${catkin_LIBRARIES} todnetworklibrary)
