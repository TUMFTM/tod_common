cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME tod_network)
project(${PROJECT_NAME})


find_package(catkin REQUIRED COMPONENTS)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES tod_network
    CATKIN_DEPENDS roscpp
    DEPENDS
    )

add_subdirectory(src/library)
add_subdirectory(src/sender)
add_subdirectory(src/receiver)

# Mark libraries for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS
    ${TOD_SENDER_TARGETS}
    ${TOD_RECEIVER_TARGETS}
    #  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    #  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY include/${PROJECT_NAME}/
    #    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    #   FILES_MATCHING PATTERN "*.h"
    #   PATTERN ".svn" EXCLUDE
    )

