cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 11)

project(xyzyaw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

############## ARA version
add_executable(xyzyaw xyzyawARA.cpp)


#target_link_libraries(xythetac ${SBPL_LIBRARIES})
target_link_libraries(xyzyaw contxyzyaw)