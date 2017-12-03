project(leg)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
	${CMAKE_SOURCE_DIR}/prj.corelibs
)

set(leg_SRC src/leg.cpp)
set(leg_HDR leg.h)

add_library(leg ${leg_SRC} ${leg_HDR})

target_link_libraries (leg pid Eigen3::Eigen)