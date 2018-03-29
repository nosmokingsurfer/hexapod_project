project(articulated_dll)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.control_dlls/articulated_dll
	${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(articulated_dll_HDR
	stdafx.h
)

set(articulated_dll_SRC
	src/articulated_dll.cpp
	src/articulated_dll.def
)

add_library(articulated_dll SHARED ${articulated_dll_SRC})

target_link_libraries(articulated_dll robot leg body linear_player Eigen3::Eigen)
set_property(TARGET articulated_dll PROPERTY FOLDER "prj.control_dlls")
