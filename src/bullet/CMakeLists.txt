cmake_minimum_required(VERSION 2.6)

include_directories("${PROJECT_SOURCE_DIR}/include")

SET (BUILD_SHARED_LIBS TRUE)

file(
        GLOB_RECURSE
        source_files
        ./*
		${PROJECT_SOURCE_DIR}/include/bullet/JointBullet.h
		${PROJECT_SOURCE_DIR}/include/bullet/Exports.h
)
 
add_library(
        bullet_integration SHARED
        ${source_files}
)