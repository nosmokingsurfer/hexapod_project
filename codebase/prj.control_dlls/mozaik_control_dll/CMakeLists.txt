project(mozaik_control_dll)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.control_dlls/mozaik_control_dll
	${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(mozaik_control_dll_HDR
	stdafx.h
)

set(mozaik_control_dll_SRC
	src/mozaik_control_dll.cpp
	src/mozaik_control_dll.def
)

add_library(mozaik_control_dll SHARED ${mozaik_control_dll_SRC})

target_link_libraries(mozaik_control_dll robot Eigen3::Eigen)
