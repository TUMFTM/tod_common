############ TOD_NETWORK_LIBRARY############
find_package(catkin REQUIRED COMPONENTS roscpp tod_msgs)
find_package(Threads REQUIRED)
find_package(PahoMqttCpp REQUIRED)


add_library(tod_network SHARED
    tod_network.cpp
    tod_sender.cpp
    tod_receiver.cpp
)

target_include_directories(tod_network PUBLIC
    ../../include
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(tod_network 
    ${catkin_LIBRARIES}
    PahoMqttCpp::paho-mqttpp3
    Threads::Threads
)

install(TARGETS tod_network
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# node for gtests
# ------------------------------------------------------------------------
# --------------------------- GTester ------------------------------------
# ------------------------------------------------------------------------
if(FALSE)
    set(TARGET_GTESTER Gtester)
    set(GTESTER_SRC
        test/Gtester.cpp
        )
    add_executable(${TARGET_GTESTER}
            ${GTESTER_SRC}
            ${TOD_NETWORK_SRC}
            )
    target_include_directories(${TARGET_GTESTER} PRIVATE
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
        src
        include
        )
    add_dependencies(${TARGET_GTESTER} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(${TARGET_GTESTER}
        PRIVATE
        ${catkin_LIBRARIES}
        PahoMqttCpp::paho-mqttpp3
        Threads::Threads
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${GTEST_LIBRARIES}
        )
    set_property(TARGET ${TARGET_GTESTER} PROPERTY CXX_STANDARD 17)
    add_dependencies(tod_network ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()
