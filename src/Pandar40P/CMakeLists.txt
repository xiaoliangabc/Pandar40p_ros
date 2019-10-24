cmake_minimum_required(VERSION 2.8)
project(Pandar40PSDK)

find_package( Boost REQUIRED )
find_package( PCL REQUIRED COMPONENTS common )

set (CMAKE_CXX_FLAGS "-fPIC --std=c++11")

add_subdirectory(src/Pandar40P)

include_directories(
    .
    include
    src/Pandar40P/include
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library( ${PROJECT_NAME} SHARED
    src/pandar40p_sdk.cc
    src/tcp_command_client.c
    src/util.c
)

target_link_libraries(${PROJECT_NAME} 
    Pandar40P
    ${Boost_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    yaml-cpp
)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(pandar40p_test
    test/test.cc
)

target_link_libraries(pandar40p_test
    Pandar40P
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    yaml-cpp
)

endif(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})