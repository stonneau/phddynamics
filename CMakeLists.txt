cmake_minimum_required(VERSION 2.6)
project(phddynamics)

string (REPLACE "/D_WINDOWS" "" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})


set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib/")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Bullet REQUIRED)
include_directories(${BULLET_SOURCE_DIR})

set(BULLETLIBS "${BULLET_LIBRARIES}")


LINK_DIRECTORIES("${BULLET_SOURCE_DIR}/../Glut"
				 "${BULLET_SOURCE_DIR}/../lib")

add_subdirectory (src/tests/pid)
add_subdirectory (src/tests/kinematics)
add_subdirectory (src/tests/bullet)

