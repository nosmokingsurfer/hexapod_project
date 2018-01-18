project(mozaik)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(dummy_SRC src/mozaik.cpp)
set(dummy_HDR mozaik.h)

add_library(mozaik ${dummy_SRC} ${dummy_HDR})

target_link_libraries (mozaik body Eigen3::Eigen)

SET_PROPERTY(TARGET mozaik PROPERTY FOLDER "prj.corelibs")