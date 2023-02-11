cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)

project(runswift)

set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake")

OPTION(FULL_DEB_INFO "Include all debug info." OFF)

IF(FULL_DEB_INFO)
    # Compile includes gdb info
    add_definitions(-Wall -O0 -g)
ELSE() # not FULL_DEB_INFO
    # Basic Compile
    add_definitions(-Wall)
ENDIF(FULL_DEB_INFO)

# Keep build process generic for v[45]
message("Toolchain File: ${CMAKE_TOOLCHAIN_FILE}")
if(CMAKE_TOOLCHAIN_FILE)
    message("CTC_DIR: ${CTC_DIR}")
    add_definitions(-DCROSS_COMPILING_FOR_ROBOT)
    if(${CTC_DIR} MATCHES atom-2.[18])
        message("Nao v4/5/6 atom build detected")
        add_definitions(-march=atom -mtune=atom -m32)
    else(${CTC_DIR} MATCHES atom-2.[18])
        message(FATAL_ERROR "Unknown CTC_DIR")
    endif(${CTC_DIR} MATCHES atom-2.[18])
else(CMAKE_TOOLCHAIN_FILE)
    add_definitions(-DNATIVE_COMPILING)
    # assume v6 for now
    add_definitions(-march=silvermont -mtune=silvermont -m64)
endif(CMAKE_TOOLCHAIN_FILE)

include("${CMAKE_CURRENT_SOURCE_DIR}/bootstrap.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/bootstrap-eigen.cmake")

add_subdirectory(robot)
add_subdirectory(utils)
