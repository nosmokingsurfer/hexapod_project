project(simple_body)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(simple_body_SRC src/simple_body.cpp)
set(simple_body_HDR simple_body.h)

add_library(simple_body ${simple_body_SRC} ${simple_body_HDR})

target_link_libraries (simple_body leg pose segment Eigen3::Eigen)

SET_PROPERTY(TARGET simple_body PROPERTY FOLDER "prj.corelibs")