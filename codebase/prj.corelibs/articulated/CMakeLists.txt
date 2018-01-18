project(articulated)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(articulated_SRC src/articulated.cpp)
set(articulated_HDR articulated.h)

add_library(articulated ${articulated_SRC} ${articulated_HDR})

target_link_libraries (articulated body Eigen3::Eigen)
set_property(TARGET articulated PROPERTY FOLDER "prj.corelibs")