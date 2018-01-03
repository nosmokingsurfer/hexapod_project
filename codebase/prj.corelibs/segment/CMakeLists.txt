project(segment)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(segment_SRC src/segment.cpp)
set(segment_HDR segment.h)

add_library(segment ${segment_SRC} ${segment_HDR})

target_link_libraries (segment leg joint Eigen3::Eigen)

set_property(TARGET segment PROPERTY FOLDER "prj.corelibs")