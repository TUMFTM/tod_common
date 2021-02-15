cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME mqtt_sender_receiver)
set(TARGET_NAME MqttSenderReceiver)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# REQUIRED FOR TOD_NETWORK
find_package(catkin REQUIRED COMPONENTS roscpp tod_network)

# REQUIRED FOR TOD_NETWORK
find_package(PahoMqttCpp REQUIRED)

catkin_package(
    INCLUDE_DIRS 
    CATKIN_DEPENDS roscpp std_msgs tod_msgs
    )

set(mqtt_node_sources
    src/MqttTransmissionBenchNode.cpp
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    # include/
    ../../devel/include # CLION HACK
    )

add_executable(${TARGET_NAME}
    ${mqtt_node_sources}
    )

add_dependencies(${TARGET_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${TARGET_NAME}
    ${catkin_LIBRARIES}     
    PahoMqttCpp::paho-mqttpp3
    )
