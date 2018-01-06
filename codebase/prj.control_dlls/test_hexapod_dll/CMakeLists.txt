project(test_hexapod_dll)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.control_dlls/test_hexapod_dll
	${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(test_hexapod_dll_HDR
	stdafx.h
)

set(test_hexapod_dll_SRC
	src/test_hexapod_dll.cpp
	src/test_hexapod_dll.def
)

add_library(test_hexapod_dll SHARED ${test_hexapod_dll_SRC})

target_link_libraries(test_hexapod_dll robot leg body Eigen3::Eigen)
set_property(TARGET test_hexapod_dll PROPERTY FOLDER "prj.control_dlls")
