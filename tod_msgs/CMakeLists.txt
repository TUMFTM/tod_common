cmake_minimum_required(VERSION 2.8.3)
project(tod_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    message_generation
    )

## Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
    FILES
    PaketInfo.msg
    StatusMsg.msg
    SafetyDriverStatusMsg.msg
    VideoInfo.msg
    )

## Generate services in the 'srv' folder
add_service_files(
    DIRECTORY srv
    FILES
    InputDevice.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    )

## The catkin_package macro generates cmake config files for your package
catkin_package(
    INCLUDE_DIRS include
    )

## Declare a C++ library
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        include)

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  #  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
