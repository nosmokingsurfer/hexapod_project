project(pose)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(pose_SRC src/pose.cpp)
set(pose_HDR pose.h)



add_library(pose ${pose_SRC} ${pose_HDR})

target_link_libraries (pose Eigen3::Eigen)

set_property(TARGET pose PROPERTY FOLDER "prj.corelibs")
