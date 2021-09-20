cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 11)

project(xyzyaw)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(HEADERS_XYZYAW
    RRTPlanner.h
)

set(SRCS
    RRTPlanner.cpp
)


include_directories( . )


add_library(planner SHARED ${SRCS} ${HEADERS_XYZYAW})

target_link_libraries(planner contxyzyaw)

target_include_directories(planner PUBLIC .)

