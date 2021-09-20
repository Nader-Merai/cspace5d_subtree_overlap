##########################################
#   continuous xyz_yaw environment
cmake_minimum_required(VERSION 3.4) 
set(CMAKE_CXX_STANDARD 11)

set(HEADERS_XYZYAW
    include/contxyzyaw/motion.h
    include/contxyzyaw/stl_reader.h
    include/contxyzyaw/environment_xyzyaw.h
    include/contxyzyaw/headers.h
    include/contxyzyaw/nanoflann.hpp
)


set(SRCS
#src/embedded_multi_heuristics.cpp
src/environment_xyzyaw.cpp
)


find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(smpl REQUIRED)
find_package(sbpl REQUIRED)


# pkg_check_modules(SBPL REQUIRED sbpl)
# pkg_check_modules(SMPL REQUIRED smpl)

include_directories( ${SBPL_INCLUDE_DIRS}
                     ${Boost_INCLUDE_DIRS}
                     ${EIGEN3_INCLUDE_DIR}
                     include/
                   )

link_directories( ${SBPL_LIBRARY_DIRS} 
                  ${SMPL_LIBRARY_DIRS}  
                )


add_library(contxyzyaw  SHARED ${SRCS} ${HEADERS_XYZYAW})

target_link_libraries(contxyzyaw PUBLIC ${SBPL_LIBRARIES} ${smpl_LIBRARIES})

target_include_directories(contxyzyaw PUBLIC .)
target_include_directories(contxyzyaw PUBLIC include)