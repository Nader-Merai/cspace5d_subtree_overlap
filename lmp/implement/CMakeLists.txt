cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 11)

project(xyzyaw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# sbpl
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(SBPL REQUIRED sbpl)
#include_directories(${SBPL_INCLUDE_DIRS})
#link_directories(${SBPL_LIBRARY_IDRS})

# contxyzyaw

##############  MHA version
#add_executable(xythetac xythetaMHA.cpp environment_navxythetaC.cpp embedded_multi_heuristics.cpp) 

############## ARA version
add_executable(xyzyaw xyzyawARA.cpp)


#target_link_libraries(xythetac ${SBPL_LIBRARIES})
target_link_libraries(xyzyaw contxyzyaw)