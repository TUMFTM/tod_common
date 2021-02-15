find_package(catkin REQUIRED COMPONENTS roscpp tod_msgs)

# ################# bitrate prediction sender ########################

set(SENDER_NAME BitratePredictionSender)
set(TOD_SENDER_TARGETS ${TOD_SENDER_TARGETS} ${SENDER_NAME})
add_executable(${SENDER_NAME}
    VehicleBitratePredictionSender.cpp
    )
target_link_libraries(${SENDER_NAME}
    ${catkin_LIBRARIES}
    tod_network
    )
target_include_directories(${SENDER_NAME}
    PRIVATE
    include)
set_property(TARGET ${SENDER_NAME} PROPERTY CXX_STANDARD 17)
