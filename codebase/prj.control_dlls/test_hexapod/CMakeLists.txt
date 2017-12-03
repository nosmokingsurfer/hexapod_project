project(test_hexapod)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.control_dlls/test_hexapod
	${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(test_hexapod_HDR
	stdafx.h
)

set(test_hexapod_SRC
	src/test_hexapod.cpp
	src/test_hexapod.def
)

add_library(test_hexapod SHARED ${test_hexapod_SRC})

target_link_libraries(test_hexapod robot leg Eigen3::Eigen)
