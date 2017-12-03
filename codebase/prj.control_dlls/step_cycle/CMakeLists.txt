project(step_cycle)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.control_dlls/step_cycle
)

set(step_cycle_HDR
	stdafx.h
)

set(step_cycle_SRC
	src/step_cycle.cpp
	src/step_cycle.def
)

add_library(step_cycle SHARED ${step_cycle_SRC})

