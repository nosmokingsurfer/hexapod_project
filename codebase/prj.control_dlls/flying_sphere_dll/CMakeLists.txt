project(flying_sphere_dll)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.control_dlls/flying_sphere_dll
)

set(flying_sphere_dll_HDR
	stdafx.h
)

set(flying_sphere_dll_SRC
	src/flying_sphere_dll.cpp
	src/flying_sphere_dll.def
)

add_library(flying_sphere_dll SHARED ${flying_sphere_dll_SRC})
set_property(TARGET flying_sphere_dll PROPERTY FOLDER "prj.control_dlls")
