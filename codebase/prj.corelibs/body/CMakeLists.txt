project(body)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(body_SRC src/body.cpp)
set(body_HDR body.h)

add_library(body ${body_SRC} ${body_HDR})

target_link_libraries (body leg pose Eigen3::Eigen)