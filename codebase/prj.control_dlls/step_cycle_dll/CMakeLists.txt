project(step_cycle_dll)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.control_dlls/step_cycle_dll
)

set(step_cycle_dll_HDR
	stdafx.h
)

set(step_cycle_dll_SRC
	src/step_cycle_dll.cpp
	src/step_cycle_dll.def
)

add_library(step_cycle_dll SHARED ${step_cycle_dll_SRC})
set_property(TARGET step_cycle_dll PROPERTY FOLDER "prj.control_dlls")

