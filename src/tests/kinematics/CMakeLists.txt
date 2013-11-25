cmake_minimum_required(VERSION 2.6)

include_directories("${PROJECT_SOURCE_DIR}/include")

add_executable(
        kinematics_test Main.cpp
		${PROJECT_SOURCE_DIR}/include/kinematics/joint.h
		${PROJECT_SOURCE_DIR}/include/kinematics/joint_io.h
)

