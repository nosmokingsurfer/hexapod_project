project(joint)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(joint_SRC src/joint.cpp)
set(joint_HDR joint.h)

add_library(joint ${joint_SRC} ${joint_HDR})

target_link_libraries (joint pid segment Eigen3::Eigen)
set_property(TARGET joint PROPERTY FOLDER "prj.corelibs")