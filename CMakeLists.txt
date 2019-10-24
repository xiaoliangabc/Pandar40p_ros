cmake_minimum_required(VERSION 2.8.3)
project(pandar40p)
find_package(catkin REQUIRED)
catkin_package(
    CATKIN_DEPENDS)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    sensor_msgs
    std_msgs
    image_transport
)

find_package( Boost REQUIRED )
find_package( PCL REQUIRED COMPONENTS common )

catkin_package()
set (CMAKE_CXX_FLAGS "-fPIC --std=c++11")

#################### Pandar40P Library
add_library(Pandar40P
        src/Pandar40P/src/Pandar40P/src/input.cc
        src/Pandar40P/src/Pandar40P/src/pandar40p_internal.cc
        src/Pandar40P/src/Pandar40P/src/pandar40p.cc
        )

target_include_directories(Pandar40P PRIVATE
        src/Pandar40P/src/Pandar40P/
        src/Pandar40P/src/Pandar40P/include
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

target_link_libraries(Pandar40P
        ${Boost_LIBRARIES}
        ${PCL_IO_LIBRARIES}
        )

#################### Pandar40PSDK

add_library( Pandar40PSDK SHARED
        src/Pandar40P/src/pandar40p_sdk.cc
        src/Pandar40P/src/tcp_command_client.c
        src/Pandar40P/src/util.c
        )

target_include_directories(Pandar40PSDK PRIVATE
        src/Pandar40P/
        src/Pandar40P/include/
        src/Pandar40P/src/Pandar40P/include/
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(Pandar40PSDK
        Pandar40P
        ${Boost_LIBRARIES}
        ${PCL_IO_LIBRARIES}
        yaml-cpp
        )

#################### ROS NODE

add_executable(pandar40p_node
        src/main.cc
        )
target_include_directories(pandar40p_node PRIVATE
        src/Pandar40P/src/Pandar40P/include
        src/Pandar40P/include
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        )

target_link_libraries(pandar40p_node
        Pandar40PSDK
        ${catkin_LIBRARIES}
        )

###############################

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(TARGETS pandar40p_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS
        Pandar40P
        Pandar40PSDK
        pandar40p_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )