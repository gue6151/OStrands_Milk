cmake_minimum_required(VERSION 3.25)

project(OStrandsMilk VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(COMMONLIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/extern/CommonLibSSE-NG")
set(ENABLE_SKYRIM_VR OFF CACHE BOOL "" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("${COMMONLIB_DIR}" CommonLibSSE EXCLUDE_FROM_ALL)
include("${COMMONLIB_DIR}/cmake/CommonLibSSE.cmake")

add_commonlibsse_plugin(
	${PROJECT_NAME}
	NAME "OStrands Milk"
	AUTHOR "OStrands milk addon"
	SOURCES
		src/main.cpp
		src/Anatomy.cpp
		src/Body.cpp
		src/FluidManager.cpp
		src/Hooks.cpp
		src/Lactation.cpp
		src/Milk.cpp
		src/Revealing.cpp
		src/Settings.cpp
		src/fluid/Clusters.cpp
		src/fluid/Ground.cpp
		src/fluid/Skin.cpp
		src/fluid/Solver.cpp
		src/fluid/Surface.cpp
)

find_path(SIMPLEINI_INCLUDE_DIRS "SimpleIni.h" REQUIRED)
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "OStrands_Milk")
target_compile_definitions(${PROJECT_NAME} PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
target_include_directories(${PROJECT_NAME} PRIVATE src extern/SKSEMenuFramework ${SIMPLEINI_INCLUDE_DIRS})
target_precompile_headers(${PROJECT_NAME} PRIVATE src/PCH.h)
