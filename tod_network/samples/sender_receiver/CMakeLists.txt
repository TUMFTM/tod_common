cmake_minimum_required(VERSION 3.10)

project(sender_receiver)

list(APPEND CMAKE_PREFIX_PATH "/opt/ros/melodic/")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    roscpp  rospy std_msgs
    )

find_package(tod_network)

catkin_package(
		CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs message_runtime
		#  DEPENDS system_lib
)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${catkin_INCLUDE_DIRS}
    )

add_executable(Sender "sender.cpp")
add_executable(Receiver "receiver.cpp")
target_link_libraries(Sender ${catkin_LIBRARIES} tod_network)
target_link_libraries(Receiver ${catkin_LIBRARIES} tod_network)
