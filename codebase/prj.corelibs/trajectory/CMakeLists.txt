project(trajectory)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(trajectory_SRC src/trajectory.cpp)
set(trajectory_HDR trajectory.h)

add_library(trajectory ${trajectory_SRC} ${trajectory_HDR})

target_link_libraries (trajectory Eigen3::Eigen)