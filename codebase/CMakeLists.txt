cmake_minimum_required(VERSION 3.4)

option(BUILD_LIBRARIES "Build hexapod libraries" ON)
option(BUILD_DLL "Build dll for UM intgeration" ON)
option(BUILD_TESTS "Build test projects" ON)


if(BUILD_TESTS)
	add_subdirectory(prj.tests)
endif()

if(BUILD_LIBRARIES)
	add_subdirectory(prj.corelibs)
endif()

if(BUILD_DLL)
	add_subdirectory(prj.control_dlls)
endif()