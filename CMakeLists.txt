cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++17 -lstdc++fs")

option(BUILD_EXAMPLE "build driver example" ON)

set(CPP_FILES 

include/cxxopts.hpp
include/generic_port.h
include/serial_port.h
include/tcp_server.h
include/udp_port.h

src/serial_port.cpp
src/tcp_server.cpp
src/udp_port.cpp
)

include_directories(include/ include/mavlink/)

add_library(communication_module ${CPP_FILES})
target_link_libraries(communication_module pthread)

if(${BUILD_EXAMPLE})
    include_directories(./)
    add_executable(communication_module_example example/main.cpp)
    target_link_libraries(communication_module_example communication_module stdc++fs)
endif()
