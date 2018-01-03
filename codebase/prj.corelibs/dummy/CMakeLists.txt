project(dummy)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(dummy_SRC src/dummy.cpp)
set(dummy_HDR dummy.h)

add_library(dummy ${dummy_SRC} ${dummy_HDR})

target_link_libraries (dummy Eigen3::Eigen)
set_property(TARGET dummy PROPERTY FOLDER "prj.corelibs")