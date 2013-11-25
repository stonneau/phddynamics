cmake_minimum_required(VERSION 2.6)


include_directories("${PROJECT_SOURCE_DIR}/include"
					"${BULLET_SOURCE_DIR}/../Demos/OpenGl"
					"${BULLET_SOURCE_DIR}/../Glut")
				
add_executable(
        bullet_tests 
		Main.cpp
		TestApplication.h
		TestApplication.cpp
)

if(MSVC10)
IF(CMAKE_CFG_INTDIR MATCHES "Debug")
target_link_libraries(
	bullet_tests 
	bullet_integration
	BulletDynamics_vs2010_x64_debug  
	BulletCollision_vs2010_x64_debug 
	LinearMath_vs2010_x64_debug  
	OpenGLSupport_vs2010_x64_debug  
	${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
ELSE ()
target_link_libraries(
	bullet_tests
	bullet_integration
	BulletDynamics_vs2010_x64_release  
	BulletCollision_vs2010_x64_release
	LinearMath_vs2010_x64_release  
	OpenGLSupport_vs2010_x64_release  
	${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
ENDIF(CMAKE_CFG_INTDIR MATCHES "Debug")
endif()

if(MSVC)
  # StringPooling: true == /GF  false == /GF-
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /GF")

  # RuntimeLibrary
  # 0 (MultiThreaded) == /MT
  # 1 (MultiThreadedDebug) == /MTd
  # 2 (MultiThreadedDLL) == /MD
  # 3 (MultiThreadedDebugDLL) == /MDd
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")

  # TreatWChar_tAsBuiltInType: true == /Zc:wchar_t  false == /Zc:wchar_t-
  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zc:wchar_t")

  # WarningLevel: /W<level 0 to 4> or /Wall
  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3")
endif()