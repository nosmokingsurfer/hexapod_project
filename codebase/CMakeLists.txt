cmake_minimum_required(VERSION 3.4)

option(BUILD_LIBRARIES "Build hexapod libraries" ON)
option(BUILD_DLL "Build dll for UM intgeration" ON)
option(BUILD_TESTS "Build test projects" ON)


if(BUILD_TESTS)
	add_subdirectory(tests)
endif()

if(BUILD_LIBRARIES)
	add_subdirectory(corelibs)
endif()

if(BUILD_DLL)
	add_subdirectory(control_dlls)
endif()