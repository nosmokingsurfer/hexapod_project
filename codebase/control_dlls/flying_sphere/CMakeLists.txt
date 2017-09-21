project(flying_sphere)

include_directories(
	${CMAKE_SOURCE_DIR}/control_dlls/flying_sphere
)

set(flying_sphere_HDR
	stdafx.h
)

set(flying_sphere_SRC
	src/flying_sphere.cpp
	src/flying_sphere.def
)

add_library(flying_sphere SHARED ${flying_sphere_SRC})

