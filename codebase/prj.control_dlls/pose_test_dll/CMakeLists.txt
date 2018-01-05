project(pose_test_dll)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.control_dlls/pose_test_dll
	${CMAKE_SOURCE_DIR}/prj.corelibs/
)

set(pose_test_dll_HDR
	stdafx.h
)

set(pose_test_dll_SRC
	src/pose_test_dll.cpp
	src/pose_test_dll.def
)

add_library(pose_test_dll SHARED ${pose_test_dll_SRC})

target_link_libraries(pose_test_dll pose Eigen3::Eigen)

set_property(TARGET pose_test_dll PROPERTY FOLDER "prj.control_dlls")

