find_package(catkin REQUIRED COMPONENTS roscpp tod_msgs)

# ################# bitrate prediction receiver ########################

set(RECEIVER_NAME BitratePredictionReceiver)
set(TOD_RECEIVER_TARGETS ${TOD_RECEIVER_TARGETS} ${RECEIVER_NAME})
add_executable(${RECEIVER_NAME}
    OperatorBitratePredictionReceiver.cpp
    )
target_link_libraries(${RECEIVER_NAME}
    ${catkin_LIBRARIES}
    tod_network
    )
target_include_directories(${RECEIVER_NAME}
    PRIVATE
    include)
set_property(TARGET ${RECEIVER_NAME} PROPERTY CXX_STANDARD 17)
