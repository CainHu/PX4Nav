cmake_minimum_required(VERSION 3.16)
project(ecl)

#set(CMAKE_BUILD_TYPE Release)

if (NOT CMAKE_BUILD_TYPE)
    message("not defined Build Type:auto define it to Debug" )
    set (CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_BUILD_TYPE)

if (CMAKE_BUILD_TYPE MATCHES "Debug" )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g ")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g ")
elseif (CMAKE_BUILD_TYPE MATCHES "Release" )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 ")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 ")
elseif (CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O2 ")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O2 ")
else()
    message("unkown defined Build Type:same handle with Debug" )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g ")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g ")
endif (CMAKE_BUILD_TYPE MATCHES "Debug")

set(ECL_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE STRING "ECL source location" FORCE)

add_definitions(-DECL_STANDALONE)
set(ECL_STANDALONE 1)

include_directories("${ECL_SOURCE_DIR}\\src")
include_directories("${ECL_SOURCE_DIR}\\src\\lib")
#include_directories("${PROJECT_SOURCE_DIR}\\src\\lib\\matrix")

add_subdirectory(src)

add_executable(ecl main.cpp)

target_link_libraries(ecl ecl_eskf)
target_link_libraries(ecl ecl_traj_sim)
target_link_libraries(ecl ecl_ahrs)
target_link_libraries(ecl ecl_geo)
target_link_libraries(ecl ecl_mag_world)
target_link_libraries(ecl ecl_inav)