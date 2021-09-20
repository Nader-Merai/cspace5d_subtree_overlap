###############################################
#   project
###############################################

cmake_minimum_required(VERSION 3.2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(contxyzyaw)
add_subdirectory(implement)
add_subdirectory(RRTPlanner)